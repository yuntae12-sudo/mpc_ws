#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

#include "cost/cost_function.hpp"
#include "global/global.hpp"
#include "global/parameter_loader.hpp"
#include "global/utils.hpp"
#include "solver/mpc_solver.hpp"
#include "udp_network/udp_manager.hpp"

namespace {

// MPC_PACKAGE_SRC_DIR은 CMakeLists.txt에서 절대경로로 주입 (실행 위치와 무관하게 동작).
constexpr const char* kMpcParamsPath = MPC_PACKAGE_SRC_DIR "/config/mpc_params.yaml";
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

// Planner 경로를 MPC 시간 간격에 맞춰 선형보간한다. 위치 최근접점이 아니라
// 시간 인덱스를 유지해야 Planner가 설계한 속도 프로파일도 함께 추종된다.
ReferencePath ToReferencePath(const PlannedPath& pp, double dst_dt, int steps) {
    ReferencePath ref;
    const size_t n = pp.size();
    if (n == 0 || pp.dt <= 0.0) return ref;

    ref.x_ref.reserve(steps);
    ref.y_ref.reserve(steps);
    ref.yaw_ref.reserve(steps);
    ref.v_ref.reserve(steps);
    ref.k_ref.reserve(steps);

    for (int i = 0; i < steps; ++i) {
        const double t = i * dst_dt;
        double idx_f = t / pp.dt;
        idx_f = clip(idx_f, 0.0, static_cast<double>(n - 1));
        const size_t i0 = static_cast<size_t>(idx_f);
        const size_t i1 = std::min(i0 + 1, n - 1);
        const double frac = idx_f - static_cast<double>(i0);

        ref.x_ref.push_back(pp.x[i0] + frac * (pp.x[i1] - pp.x[i0]));
        ref.y_ref.push_back(pp.y[i0] + frac * (pp.y[i1] - pp.y[i0]));
        ref.yaw_ref.push_back(pp.yaw[i0] + frac * angleDiff(pp.yaw[i1], pp.yaw[i0]));
        ref.v_ref.push_back(pp.v[i0] + frac * (pp.v[i1] - pp.v[i0]));
        ref.k_ref.push_back(pp.kappa[i0] + frac * (pp.kappa[i1] - pp.kappa[i0]));
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

    int stuck_cycles = 0;
    int recovery_cycles_left = 0;
    const int stuck_timeout_cycles = std::max(1, static_cast<int>(
        std::ceil(mpc_params.stuck_timeout * freq)));
    const int recovery_duration_cycles = std::max(1, static_cast<int>(
        std::ceil(mpc_params.stuck_recovery_duration * freq)));

    while (true) {
        const auto t0 = std::chrono::steady_clock::now();

        // Planner 경로를 받기 전에는 정지 명령을 유지한다.
        if (!udp.has_planned_path()) {
            std::printf("[MPC] Waiting for planned path...\n");
            udp.send_ctrl_cmd(makeStopCommand());
            std::this_thread::sleep_until(t0 + period);
            continue;
        }

        const PlannedPath pp = udp.get_planned_path();

        // Planner와 Controller가 동일 상태를 사용하도록 PlannedPath의 Ego snapshot을 사용한다.
        MPCState ego_snap;
        ego_snap.x = pp.ego_x;
        ego_snap.y = pp.ego_y;
        ego_snap.yaw = pp.ego_yaw;
        ego_snap.vx = pp.ego_v;
        ego = ego_snap;

        // 2) PlannedPath -> ReferencePath (시간 정렬)
        ReferencePath ref = ToReferencePath(pp, mpc_params.dt, mpc_params.horizon);

        // Behavior mode와 무관한 정지 데드락 판정. 정상 STOP/WAIT는 max_ref_speed가
        // 0이고, 한 점 정지 경로는 path_progress가 0이므로 절대 진입하지 않는다.
        double max_ref_speed = 0.0;
        for (double v_ref : ref.v_ref) max_ref_speed = std::max(max_ref_speed, v_ref);
        double path_progress = 0.0;
        if (ref.size() >= 2) {
            path_progress = std::hypot(ref.x_ref.back() - ref.x_ref.front(),
                                       ref.y_ref.back() - ref.y_ref.front());
        }
        const bool forward_intent = max_ref_speed > mpc_params.stuck_target_speed &&
                                    path_progress > mpc_params.stuck_min_path_progress;
        const bool ego_stopped = std::fabs(ego_snap.vx) < mpc_params.stuck_ego_speed;

        if (!forward_intent || !ego_stopped) {
            stuck_cycles = 0;
            recovery_cycles_left = 0;
        } else if (recovery_cycles_left == 0 && ++stuck_cycles >= stuck_timeout_cycles) {
            // 프로세스 재실행이 효과가 있었던 핵심 상태만 명시적으로 초기화한다.
            // 경로/조향 계획은 유지하고, 이전 제동해가 새 최적화를 붙잡지 않게 한다.
            warm_start.clear();
            last_control = MPCControl{};
            recovery_cycles_left = recovery_duration_cycles;
            stuck_cycles = 0;
            std::printf("[MPC-STUCK] recovery start: ego_v=%.2f max_ref_v=%.2f "
                        "path_progress=%.2f duration=%.2fs\n",
                        ego_snap.vx, max_ref_speed, path_progress,
                        mpc_params.stuck_recovery_duration);
        }

        // 3) MPC 풀기
        const MPCControl prev_control_for_debug = last_control;  // rate cost 진단용 (아래 breakdown)
        MPCResult res = solveMPC(ego_snap, ref, last_control, warm_start, mpc_params);

        if (!res.success) {
            std::printf("[MPC] solver msg: %s\n", res.solver_msg.c_str());
        }

        // 큰 종방향 입력의 원인을 확인하기 위한 cost 진단 로그.
        if (std::fabs(res.control.accel) > 1.5) {
            CostBreakdown bd = computeCostBreakdown(res.predicted_states, res.controls, ref,
                                                     prev_control_for_debug, mpc_params);
            std::printf("[MPC-COST] path=%.2f heading=%.2f speed=%.2f control=%.2f rate=%.2f terminal=%.2f | accel=%.2f\n",
                        bd.path, bd.heading, bd.speed, bd.control, bd.control_rate, bd.terminal,
                        res.control.accel);
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

        double accel_raw = res.control.accel;
        if (recovery_cycles_left > 0 && forward_intent && ego_stopped) {
            // Planner가 충돌검증한 전진 경로가 있을 때만 정지마찰/solver 국소해를
            // 벗어날 최소 가속을 한정된 시간 동안 보장한다.
            accel_raw = std::max(accel_raw, mpc_params.stuck_launch_accel);
            // 다음 최적화의 rate cost도 실제 차량에 보낸 입력을 기준으로 해야
            // 복구 가속과 내부의 과거 제동 입력이 서로 싸우지 않는다.
            last_control.accel = accel_raw;
            --recovery_cycles_left;
            if (recovery_cycles_left == 0) {
                std::printf("[MPC-STUCK] recovery pulse complete; normal MPC resumed\n");
            }
        }
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

        std::this_thread::sleep_until(t0 + period);
    }

    return 0;
}
