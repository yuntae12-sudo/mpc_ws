#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

#include "frenet_planner/frenet_planner.hpp"
#include "udp_network/udp_manager.hpp"

namespace {

// FRENET_PACKAGE_SRC_DIR은 CMakeLists.txt에서 절대경로로 주입 (실행 위치와 무관하게 동작).
constexpr const char* kFrenetParamsPath = FRENET_PACKAGE_SRC_DIR "/frenet_planner/config/params.yaml";
constexpr const char* kNetworkConfigPath = FRENET_PACKAGE_SRC_DIR "/udp_network/network.yaml";

constexpr double kLoopFreqHz = 20.0;  // mpc_node와 동일 주기

double NormalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

// Plan() 실패가 오래 지속될 때 mpc_node로 보내는 "정지" 경로. 한 점(count=1,
// v=0)만 있으면 mpc_controller의 ToReferencePath가 그 점으로 horizon 전체를
// 채우게 되어(idx_f가 0으로 clip) 결과적으로 "여기서 정지"를 명령하는 것과
// 동일해진다 - 기존 mpc_controller/main.cpp의 makeStopCommand()가 하던
// 역할을 경로 레벨에서 대신한다.
CartesianPath MakeStopPath(const CartesianState& ego) {
    CartesianPath cp;
    cp.x = {ego.x};
    cp.y = {ego.y};
    cp.yaw = {ego.yaw};
    cp.kappa = {0.0};
    cp.v = {0.0};
    cp.a = {0.0};
    return cp;
}

}  // namespace

int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    std::printf("========================================\n");
    std::printf("    Frenet Planner Node Starting\n");
    std::printf("========================================\n");

    FrenetPlanner frenet_planner;
    if (!frenet_planner.Init(kFrenetParamsPath)) {
        std::printf("[FrenetPlanner] Init failed. Exiting.\n");
        return 1;
    }

    std::unique_ptr<UdpManager> udp_ptr;
    try {
        udp_ptr = std::make_unique<UdpManager>(kNetworkConfigPath);
    } catch (const std::exception& e) {
        std::printf("[FrenetPlanner] UDP init failed: %s\n", e.what());
        std::printf("[FrenetPlanner] 1024 미만 포트는 root 권한이 필요합니다 (sudo로 실행).\n");
        return 1;
    }
    UdpManager& udp = *udp_ptr;

    const auto period = std::chrono::duration<double>(1.0 / kLoopFreqHz);

    // Plan() 실패(주로 저속 경계에서 곡률필터에 후보가 0개 걸리는 일시적 현상)마다
    // 매번 정지 경로를 보내면 몇 사이클만 연속 실패해도 mpc_node 쪽에서 급브레이크가
    // 누적된다(mpc_controller/main.cpp에서 실측 재현했던 문제, 오늘 이 파일로
    // 옮겨옴). 짧은 연속 실패는 마지막으로 성공했던 경로를 그대로 재전송해
    // mpc_node의 MPC가 계속 능동적으로 그 경로를 향해 보정하게 하고, 그 이상
    // 계속 실패할 때만(진짜 위험하다고 보고) 정지 경로로 넘어간다.
    constexpr int kMaxTransientFailCycles = 5;  // 20Hz 기준 0.25초
    int plan_fail_streak = 0;
    CartesianPath last_cp;
    bool has_last_cp = false;
    VehicleState last_vehicle_state;
    bool has_last_vehicle_state = false;

    std::printf("[FrenetPlanner] Control loop @ %.1f Hz\n", kLoopFreqHz);
    std::printf("========================================\n");

    while (true) {
        const auto t0 = std::chrono::steady_clock::now();

        if (!udp.has_vehicle_state()) {
            static int localization_wait_tick = 0;
            if (++localization_wait_tick % 20 == 0) {
                std::printf("[FrenetPlanner] Waiting for %s vehicle state...\n",
                            udp.using_gps_imu() ? "fresh GPS/IMU" : "ego UDP");
            }
            // 선택 localization이 주행 중 timeout되어도 MPC가 마지막 주행경로를
            // 계속 추종하지 않도록 마지막 유효 pose에서 v=0 정지경로를 갱신한다.
            if (has_last_vehicle_state) {
                CartesianState stale_ego;
                stale_ego.x = last_vehicle_state.x;
                stale_ego.y = last_vehicle_state.y;
                stale_ego.yaw = last_vehicle_state.yaw;
                const CartesianPath stop = MakeStopPath(stale_ego);
                udp.send_planned_path(stop, frenet_planner.sample_dt(),
                                      frenet_planner.last_d(), 0.0,
                                      last_vehicle_state);
            }
            std::this_thread::sleep_until(t0 + period);
            continue;
        }

        const VehicleState vs = udp.get_vehicle_state();
        last_vehicle_state = vs;
        has_last_vehicle_state = true;

        // GPS/IMU 전환 전 shadow-mode 비교: Planner/MPC에는 계속 MORAI
        // EgoVehicleStatus(vs)를 사용하고, 센서 기반 pose는 1Hz로 오차만 출력한다.
        // 이 단계에서 좌표 원점/yaw 축/속도 단위를 검증한 뒤 입력을 전환한다.
        static int localization_log_tick = 0;
        if (++localization_log_tick % 20 == 0) {
            if (udp.has_gps_imu_pose() && udp.has_ego_vehicle_state()) {
                const VehicleState sensor = udp.get_gps_imu_pose();
                const VehicleState truth = udp.get_ego_vehicle_state();
                const double dx = sensor.x - truth.x;
                const double dy = sensor.y - truth.y;
                const double dyaw = NormalizeAngle(sensor.yaw - truth.yaw);
                std::printf("[LOCALIZATION-CHECK] ego=(%.3f,%.3f yaw=%.4f v=%.3f) "
                            "gps_imu=(%.3f,%.3f yaw=%.4f v=%.3f) "
                            "error=(dx=%.3f dy=%.3f pos=%.3f dyaw=%.4f dv=%.3f)\n",
                            truth.x, truth.y, truth.yaw, truth.v,
                            sensor.x, sensor.y, sensor.yaw, sensor.v,
                            dx, dy, std::hypot(dx, dy), dyaw, sensor.v - truth.v);
            } else {
                std::printf("[LOCALIZATION-CHECK] waiting for valid GPS position and IMU data\n");
            }
        }

        // TODO: ego_cs.kappa를 vs.steer 기반으로 추정하려 했으나(mpc_controller
        // 쪽 동일 TODO 참고), steer 단위/부호가 실측 검증 전이라 CartesianToFrenet의
        // d_pprime 계산이 폭주할 위험이 있어 0(순간 직진 가정)으로 안전하게 둔다.
        CartesianState ego_cs;
        ego_cs.x = vs.x;
        ego_cs.y = vs.y;
        ego_cs.yaw = vs.yaw;
        ego_cs.kappa = 0.0;
        ego_cs.v = vs.v;
        ego_cs.a = vs.accel;

        const std::vector<ObjectInfo> obstacles = udp.get_objects();

        CartesianPath cp;
        const bool plan_ok = frenet_planner.Plan(ego_cs, obstacles, cp);

        CartesianPath* send_cp = nullptr;
        if (plan_ok) {
            plan_fail_streak = 0;
            last_cp = cp;
            has_last_cp = true;
            send_cp = &cp;
        } else {
            ++plan_fail_streak;
            const bool collision_blocked =
                frenet_planner.last_failure_reason() ==
                    FrenetPlanner::PlanFailureReason::COLLISION_BLOCKED;
            if (collision_blocked) {
                // 새 후보가 전부 충돌로 제거됐는데 과거 주행 경로를 재사용하면
                // 이미 위험해진 경로를 그대로 따라가게 된다. streak 유예 없이
                // 현재 pose 기준 정지 경로로 즉시 교체한다.
                std::printf("[FrenetPlanner] collision blocked: discarding last path, "
                            "sending stop path immediately\n");
                last_cp = MakeStopPath(ego_cs);
                has_last_cp = true;
                send_cp = &last_cp;
            } else if (has_last_cp && plan_fail_streak <= kMaxTransientFailCycles) {
                std::printf("[FrenetPlanner] no valid path (transient %d/%d, "
                            "resending last valid path)\n",
                            plan_fail_streak, kMaxTransientFailCycles);
                send_cp = &last_cp;
            } else {
                std::printf("[FrenetPlanner] no valid path (%d consecutive, "
                            "sending stop path)\n",
                            plan_fail_streak);
                last_cp = MakeStopPath(ego_cs);
                has_last_cp = true;
                send_cp = &last_cp;
            }
        }

        udp.send_planned_path(*send_cp, frenet_planner.sample_dt(),
                               frenet_planner.last_d(), frenet_planner.last_d_dot(), vs);

        std::this_thread::sleep_until(t0 + period);
    }

    return 0;
}
