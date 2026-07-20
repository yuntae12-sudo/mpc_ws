#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

#include "frenet_planner/frenet_planner.hpp"
#include "global/global.hpp"
#include "global/parameter_loader.hpp"
#include "global/utils.hpp"
#include "solver/mpc_solver.hpp"
#include "udp_network/udp_manager.hpp"

namespace {

// MPC_PACKAGE_SRC_DIR은 CMakeLists.txt에서 절대경로로 주입 (실행 위치와 무관하게 동작).
constexpr const char* kMpcParamsPath = MPC_PACKAGE_SRC_DIR "/config/mpc_params.yaml";
constexpr const char* kFrenetParamsPath = MPC_PACKAGE_SRC_DIR "/frenet_planner/config/params.yaml";
constexpr const char* kNetworkConfigPath = MPC_PACKAGE_SRC_DIR "/udp_network/network.yaml";

CtrlCmd makeStopCommand() {
    CtrlCmd cmd;
    cmd.ctrl_mode = 2;
    cmd.gear = 4;
    cmd.cmd_type = 1;
    cmd.accel = 0.0f;
    cmd.brake = 0.3f;
    cmd.steer = 0.0f;
    return cmd;
}

// FrenetPlanner의 CartesianPath(샘플 간격 src_dt) -> mpc의 ReferencePath.
//
// cp는 매 사이클 "지금 이 순간"의 ego 상태에서 새로 생성되는 궤적이므로,
// MPC 쪽에서 위치 기준으로 가장 가까운 ref 점을 찾는 방식(closest-point
// tracking)을 쓰면 안 된다: 정지 상태에서 시작하면 cp[0]의 목표 속도도
// 0이라서, "가만히 있기"가 위치오차/속도오차 모두 0인 완벽한 해가 되어
// 절대 출발하지 못하는 국소최소값에 빠진다(실측 재현 완료). cp는 이미
// 시간에 따라 균일 샘플링돼 있으므로, MPC의 각 스텝 i(시간 i*dst_dt)에
// 대응하는 cp 시점을 선형보간해 시간 정렬된 궤적으로 맞춰 넘긴다.
ReferencePath ToReferencePath(const CartesianPath& cp, double src_dt, double dst_dt, int steps) {
    ReferencePath ref;
    const size_t n = cp.x.size();
    if (n == 0 || src_dt <= 0.0) return ref;

    ref.x_ref.reserve(steps);
    ref.y_ref.reserve(steps);
    ref.yaw_ref.reserve(steps);
    ref.v_ref.reserve(steps);
    ref.k_ref.reserve(steps);

    for (int i = 0; i < steps; ++i) {
        const double t = i * dst_dt;
        double idx_f = t / src_dt;
        idx_f = clip(idx_f, 0.0, static_cast<double>(n - 1));
        const size_t i0 = static_cast<size_t>(idx_f);
        const size_t i1 = std::min(i0 + 1, n - 1);
        const double frac = idx_f - static_cast<double>(i0);

        ref.x_ref.push_back(cp.x[i0] + frac * (cp.x[i1] - cp.x[i0]));
        ref.y_ref.push_back(cp.y[i0] + frac * (cp.y[i1] - cp.y[i0]));
        ref.yaw_ref.push_back(cp.yaw[i0] + frac * angleDiff(cp.yaw[i1], cp.yaw[i0]));
        ref.v_ref.push_back(cp.v[i0] + frac * (cp.v[i1] - cp.v[i0]));
        ref.k_ref.push_back(cp.kappa[i0] + frac * (cp.kappa[i1] - cp.kappa[i0]));
    }
    return ref;
}

}  // namespace

int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    std::printf("========================================\n");
    std::printf("    MPC UDP Standalone Starting\n");
    std::printf("========================================\n");

    loadMPCParameters(kMpcParamsPath);

    FrenetPlanner frenet_planner;
    if (!frenet_planner.Init(kFrenetParamsPath)) {
        std::printf("[MPC] FrenetPlanner init failed. Node will keep running but will send stop commands.\n");
    }

    ego.x = 0.0; ego.y = 0.0; ego.yaw = 0.0; ego.vx = 0.0;
    last_control.delta = 0.0; last_control.accel = 0.0;

    std::unique_ptr<UdpManager> udp_ptr;
    try {
        udp_ptr = std::make_unique<UdpManager>(kNetworkConfigPath);
    } catch (const std::exception& e) {
        std::printf("[MPC] UDP init failed: %s\n", e.what());
        std::printf("[MPC] 1024 미만 포트는 root 권한이 필요합니다 (sudo로 실행하거나 setcap 사용).\n");
        return 1;
    }
    UdpManager& udp = *udp_ptr;

    std::vector<MPCControl> warm_start;
    double freq = mpc_params.control_frequency > 0.0 ? mpc_params.control_frequency : 10.0;
    const auto period = std::chrono::duration<double>(1.0 / freq);

    std::printf("[MPC] Control loop @ %.1f Hz\n", freq);
    std::printf("========================================\n");

    while (true) {
        const auto t0 = std::chrono::steady_clock::now();

        if (!udp.has_vehicle_state()) {
            std::printf("[MPC] Waiting for ego_vehicle status...\n");
            udp.send_ctrl_cmd(makeStopCommand());
            std::this_thread::sleep_until(t0 + period);
            continue;
        }

        // 1) ego 스냅샷: EgoVehicleStatus(ground truth) 그대로 사용.
        const VehicleState vs = udp.get_vehicle_state();
        MPCState ego_snap;
        ego_snap.x = vs.x;
        ego_snap.y = vs.y;
        ego_snap.yaw = vs.yaw;
        ego_snap.vx = vs.v;
        ego = ego_snap;

        // FrenetPlanner 입력용 CartesianState.
        // TODO: ego_cs.kappa를 vs.steer(단위/부호 미검증, udp_network/datatypes.hpp 참고)
        // 기반 tan(steer)/wheelbase로 추정했었으나, 단위가 틀리면 CartesianToFrenet의
        // d_pprime 계산이 폭주해 모든 후보가 max_lateral_accel을 넘어 무효화되는 문제가
        // 있었음. steer 실측 검증 전까지는 0(순간 직진 가정)으로 안전하게 둔다.
        CartesianState ego_cs;
        ego_cs.x = vs.x;
        ego_cs.y = vs.y;
        ego_cs.yaw = vs.yaw;
        ego_cs.kappa = 0.0;
        ego_cs.v = vs.v;
        ego_cs.a = vs.accel;

        // 2) FrenetPlanner: 전역 경로 기반 Local trajectory 생성 (LANE_KEEPING 고정)
        CartesianPath cp;
        const std::vector<ObjectInfo> obstacles = udp.get_objects();
        static int obstacle_log_counter = 0;
        if (++obstacle_log_counter % 20 == 0) {  // 1초(20Hz)에 한 번만 - 로그 스팸 방지
            std::printf("[MPC-DEBUG] has_object_info=%d obstacles=%zu",
                        udp.has_object_info(), obstacles.size());
            if (!obstacles.empty()) {
                std::printf(" first: id=%d x=%.2f y=%.2f w=%.2f l=%.2f",
                            obstacles[0].id, obstacles[0].x, obstacles[0].y,
                            obstacles[0].width, obstacles[0].length);
            }
            std::printf("\n");
        }
        if (!frenet_planner.Plan(ego_cs, obstacles, cp)) {
            std::printf("[MPC] FrenetPlanner: no valid path\n");
            udp.send_ctrl_cmd(makeStopCommand());
            std::this_thread::sleep_until(t0 + period);
            continue;
        }
        ReferencePath ref = ToReferencePath(cp, frenet_planner.sample_dt(),
                                            mpc_params.dt, mpc_params.horizon);

        // 3) MPC 풀기
        MPCResult res = solveMPC(ego_snap, ref, last_control, warm_start, mpc_params);

        if (!res.success) {
            std::printf("[MPC] solver msg: %s\n", res.solver_msg.c_str());
        }

        // 4) warm-start 갱신
        if (res.success && res.controls.size() == static_cast<size_t>(mpc_params.horizon)) {
            warm_start = res.controls;
            last_control = res.control;
        } else {
            std::printf("[MPC WS KEEP] keep previous warm_start. success=%d controls=%zu horizon=%d\n",
                        res.success, res.controls.size(), mpc_params.horizon);
        }

        // 5) 제어 출력 변환 및 송신
        const double steer_rad = clip(res.control.delta, -mpc_params.steering_max, mpc_params.steering_max);
        const double steer_norm = clip(steer_rad / mpc_params.steering_max, -1.0, 1.0);

        const double accel_raw = res.control.accel;
        double accel_norm = 0.0, brake_norm = 0.0;
        if (accel_raw >= 0.0) {
            accel_norm = clip(accel_raw / mpc_params.accel_max, 0.0, 1.0);
        } else {
            brake_norm = clip(-accel_raw / std::fabs(mpc_params.accel_min), 0.0, 1.0);
        }

        CtrlCmd cmd;
        cmd.ctrl_mode = 2;
        cmd.gear = 4;
        cmd.cmd_type = 1;
        cmd.accel = static_cast<float>(accel_norm);
        cmd.brake = static_cast<float>(brake_norm);
        cmd.steer = static_cast<float>(steer_norm);
        udp.send_ctrl_cmd(cmd);

        std::printf("[MPC] pos=(%.2f,%.2f) yaw=%.2f vx=%.2f | steer=%.3f rad | accel_raw=%.3f m/s2 | accel=%.2f brake=%.2f | cost=%.2f\n",
                    ego_snap.x, ego_snap.y, ego_snap.yaw, ego_snap.vx,
                    steer_rad, accel_raw, accel_norm, brake_norm, res.cost);

        std::this_thread::sleep_until(t0 + period);
    }

    return 0;
}
