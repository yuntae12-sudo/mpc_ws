#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "frenet_planner/global/global.hpp"
#include "udp_network/datatypes.hpp"
#include "udp_network/receiver/ego_info_receiver.hpp"
#include "udp_network/receiver/gps_receiver.hpp"
#include "udp_network/receiver/imu_receiver.hpp"
#include "udp_network/receiver/object_info_receiver.hpp"
#include "udp_network/sender/planned_path_sender.hpp"

// network.yaml 로드 + 소켓 오픈을 담당. main.cpp는 이 클래스를 통해서만
// UDP를 다룬다 (mpc_controller의 UdpManager와 같은 역할 분리 - 알고리즘은
// MORAI/자체 패킷 포맷을 몰라야 한다).
class UdpManager {
public:
    explicit UdpManager(const std::string& config_path);

    bool has_vehicle_state() const;
    VehicleState get_vehicle_state() const;
    bool has_ego_vehicle_state() const;
    VehicleState get_ego_vehicle_state() const;
    bool using_gps_imu() const { return use_gps_imu_; }

    bool has_object_info() const;
    std::vector<ObjectInfo> get_objects() const;

    // GPS/IMU 검증용 - main.cpp의 실제 ego pose 소스로는 아직 연동 안 됨(그대로
    // ego_info_receiver_ 사용). GPS(위치+속도)와 IMU(yaw)를 조합해 VehicleState
    // 모양으로 반환한다. GPS가 아직 fix를 못 잡았으면 has_gps_imu_pose()==false.
    bool has_gps_imu_pose() const;
    VehicleState get_gps_imu_pose() const;

    // mpc_node로 계산된 경로 + 중계용 ego 상태 + 진단값(d/d_dot)을 보낸다.
    void send_planned_path(const CartesianPath& cp, double dt, double d, double d_dot,
                            const VehicleState& ego);

private:
    std::unique_ptr<EgoInfoReceiver> ego_info_receiver_;
    std::unique_ptr<ObjectInfoReceiver> object_info_receiver_;
    std::unique_ptr<GpsReceiver> gps_receiver_;
    std::unique_ptr<ImuReceiver> imu_receiver_;
    std::unique_ptr<PlannedPathSender> planned_path_sender_;

    double gps_utm_east_offset_ = 0.0;
    double gps_utm_north_offset_ = 0.0;
    double gps_timeout_s_ = 0.5;
    double imu_timeout_s_ = 0.15;
    double max_prediction_time_s_ = 0.5;
    bool use_gps_imu_ = false;

    // 20Hz 연속 localization 상태. GPS는 correction으로만 사용하고 매 호출 사이에는
    // 최신 IMU yaw/추정 속도로 propagation한다.
    mutable std::mutex localization_mutex_;
    mutable bool localization_initialized_ = false;
    mutable double localization_x_ = 0.0;
    mutable double localization_y_ = 0.0;
    mutable double localization_v_ = 0.0;
    mutable uint64_t localization_update_ns_ = 0;
    mutable uint64_t applied_gps_position_ns_ = 0;
};
