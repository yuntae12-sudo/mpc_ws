#include "udp_network/udp_manager.hpp"

#include <cstdio>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "udp_network/gps_transform.hpp"

namespace {
uint64_t MonotonicNowNs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

double AgeSeconds(uint64_t now_ns, uint64_t stamp_ns) {
    if (stamp_ns == 0 || now_ns < stamp_ns) return 1e9;
    return static_cast<double>(now_ns - stamp_ns) * 1e-9;
}
}  // namespace

UdpManager::UdpManager(const std::string& config_path) {
    YAML::Node root = YAML::LoadFile(config_path);
    YAML::Node network = root["network"];

    const std::string host_ip = network["host_ip"].as<std::string>();
    const std::string mpc_ip = network["mpc_ip"].as<std::string>();
    const int ego_vehicle_dst_port = network["ego_vehicle_dst_port"].as<int>();
    const int object_info_dst_port = network["object_info_dst_port"].as<int>();
    const int gps_dst_port = network["gps_dst_port"].as<int>();
    const int imu_dst_port = network["imu_dst_port"].as<int>();
    const int planned_path_dst_port = network["planned_path_dst_port"].as<int>();

    YAML::Node gps_offset = network["gps_utm_offset"];
    gps_utm_east_offset_ = gps_offset["east"].as<double>();
    gps_utm_north_offset_ = gps_offset["north"].as<double>();
    YAML::Node localization = network["localization"];
    const std::string localization_source = localization["source"].as<std::string>();
    if (localization_source == "gps_imu") {
        use_gps_imu_ = true;
    } else if (localization_source == "ego_udp") {
        use_gps_imu_ = false;
    } else {
        throw std::runtime_error("Unknown localization.source: " + localization_source);
    }
    gps_timeout_s_ = localization["gps_timeout"].as<double>();
    imu_timeout_s_ = localization["imu_timeout"].as<double>();
    max_prediction_time_s_ = localization["max_prediction_time"].as<double>();

    std::printf("[UDP] ego_vehicle recv    <- %s:%d\n", host_ip.c_str(), ego_vehicle_dst_port);
    std::printf("[UDP] object_info recv    <- %s:%d\n", host_ip.c_str(), object_info_dst_port);
    std::printf("[UDP] gps recv            <- %s:%d\n", host_ip.c_str(), gps_dst_port);
    std::printf("[UDP] imu recv            <- %s:%d\n", host_ip.c_str(), imu_dst_port);
    std::printf("[UDP] planned_path send   -> %s:%d\n", mpc_ip.c_str(), planned_path_dst_port);
    std::printf("[Localization] source     = %s\n",
                use_gps_imu_ ? "gps_imu" : "ego_udp");

    ego_info_receiver_ = std::make_unique<EgoInfoReceiver>(host_ip, ego_vehicle_dst_port);
    object_info_receiver_ = std::make_unique<ObjectInfoReceiver>(host_ip, object_info_dst_port);
    gps_receiver_ = std::make_unique<GpsReceiver>(host_ip, gps_dst_port);
    imu_receiver_ = std::make_unique<ImuReceiver>(host_ip, imu_dst_port);
    planned_path_sender_ = std::make_unique<PlannedPathSender>(mpc_ip, planned_path_dst_port);
}

bool UdpManager::has_vehicle_state() const {
    return use_gps_imu_ ? has_gps_imu_pose() : ego_info_receiver_->has_data();
}

VehicleState UdpManager::get_vehicle_state() const {
    return use_gps_imu_ ? get_gps_imu_pose() : ego_info_receiver_->get_state();
}

bool UdpManager::has_ego_vehicle_state() const {
    return ego_info_receiver_->has_data();
}

VehicleState UdpManager::get_ego_vehicle_state() const {
    return ego_info_receiver_->get_state();
}

bool UdpManager::has_object_info() const {
    return object_info_receiver_->has_data();
}

std::vector<ObjectInfo> UdpManager::get_objects() const {
    return object_info_receiver_->get_objects();
}

bool UdpManager::has_gps_imu_pose() const {
    if (!gps_receiver_->has_data() || !imu_receiver_->has_data()) return false;
    const GpsFix fix = gps_receiver_->get_fix();
    const ImuData imu = imu_receiver_->get_data();
    const uint64_t now_ns = MonotonicNowNs();
    return fix.has_position &&
           AgeSeconds(now_ns, fix.receive_time_ns) <= gps_timeout_s_ &&
           AgeSeconds(now_ns, imu.receive_time_ns) <= imu_timeout_s_;
}

VehicleState UdpManager::get_gps_imu_pose() const {
    const GpsFix fix = gps_receiver_->get_fix();
    const ImuData imu = imu_receiver_->get_data();

    VehicleState state;
    double easting, northing;
    GpsToUtm52N(fix.lat_deg, fix.lon_deg, easting, northing);
    state.yaw = QuaternionToYaw(imu.ori_w, imu.ori_x, imu.ori_y, imu.ori_z);
    const uint64_t now_ns = MonotonicNowNs();
    const double gps_x = easting - gps_utm_east_offset_;
    const double gps_y = northing - gps_utm_north_offset_;

    std::lock_guard<std::mutex> lock(localization_mutex_);
    if (!localization_initialized_) {
        localization_x_ = gps_x;
        localization_y_ = gps_y;
        localization_v_ = fix.speed_mps;
        localization_update_ns_ = now_ns;
        applied_gps_position_ns_ = fix.position_time_ns;
        localization_initialized_ = true;
    } else {
        const double dt = std::min(max_prediction_time_s_,
            AgeSeconds(now_ns, localization_update_ns_));
        if (dt > 0.0 && dt < 1e8) {
            localization_x_ += localization_v_ * std::cos(state.yaw) * dt;
            localization_y_ += localization_v_ * std::sin(state.yaw) * dt;
        }
        localization_update_ns_ = now_ns;

        if (fix.position_time_ns != 0 &&
            fix.position_time_ns != applied_gps_position_ns_) {
            const double innovation_x = gps_x - localization_x_;
            const double innovation_y = gps_y - localization_y_;
            const double forward = innovation_x * std::cos(state.yaw) +
                                   innovation_y * std::sin(state.yaw);
            const double lateral = -innovation_x * std::sin(state.yaw) +
                                    innovation_y * std::cos(state.yaw);
            // GPS correction 하나 때문에 진행방향 상태가 뒤로 점프하거나 d가 크게
            // 튀지 않도록 body-frame innovation을 물리적인 범위로 제한한다.
            const double backward_limit = fix.stationary_confirmed ? -0.30 : -0.05;
            const double limited_forward = std::max(backward_limit, std::min(forward, 0.75));
            const double limited_lateral = std::max(-0.30, std::min(lateral, 0.30));
            constexpr double kPositionCorrectionGain = 0.45;
            localization_x_ += kPositionCorrectionGain *
                (limited_forward * std::cos(state.yaw) - limited_lateral * std::sin(state.yaw));
            localization_y_ += kPositionCorrectionGain *
                (limited_forward * std::sin(state.yaw) + limited_lateral * std::cos(state.yaw));

            if (fix.stationary_confirmed) {
                localization_v_ = 0.0;
            } else {
                // GpsReceiver에서 이미 중복 fix 제거와 저역통과를 마친 속도다.
                // 여기서 다시 저역통과하면 정지 응답이 두 번 지연되므로 새 fix의
                // 속도를 연속 propagation 상태에 직접 반영한다.
                localization_v_ = std::max(0.0, fix.speed_mps);
            }
            applied_gps_position_ns_ = fix.position_time_ns;
        } else if (fix.stationary_confirmed) {
            localization_v_ = 0.0;
        }
    }
    state.x = localization_x_;
    state.y = localization_y_;
    state.v = localization_v_;
    // steer/accel은 GPS/IMU 원시값에서 직접 못 얻는다(steer는 조향계 센서 필요,
    // accel은 IMU 장착 방향에 대한 검증 없이 lin_acc_x를 그대로 쓰면 부호/축이
    // 틀릴 위험) - 검증 전까지 0으로 둔다.
    state.steer = 0.0;
    state.accel = 0.0;
    return state;
}

void UdpManager::send_planned_path(const CartesianPath& cp, double dt, double d, double d_dot,
                                    const VehicleState& ego) {
    planned_path_sender_->Send(cp, dt, d, d_dot, ego);
}
