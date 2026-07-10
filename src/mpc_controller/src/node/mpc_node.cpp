#include "mpc_node.hpp"
#include "../global/utils.hpp"

// ========================================
// 솔버 warm-start 보존용 (이 파일에 한정)
// ========================================
static std::vector<MPCControl> g_warm_start;

// ========================================
// ROS 콜백 (변경 없음)
// ========================================

// EgoVehicleStatus.position/heading을 GPS+IMU 변환 없이 그대로 ego 상태로
// 사용한다 (MORAI 월드좌표계, 이번 단계에서는 waypoint_file은 아직 GPS-ENU
// 좌표계라 서로 프레임이 다름 — 다음 단계에서 mgeo 기반 경로로 교체 예정).
void CBEgoState(const morai_msgs::EgoVehicleStatus::ConstPtr& msg)
{
    if (!msg) return;
    std::lock_guard<std::mutex> lk(ego_mutex);
    ego.x   = msg->position.x;
    ego.y   = msg->position.y;
    ego.yaw = deg2rad(msg->heading);
    ego.vx  = msg->velocity.x;
    ego_received = true;
}

// planner의 CartesianPath 발행 레이아웃 [n, x[n], y[n], yaw[n], kappa[n], v[n], a[n]]을
// 그대로 파싱한다 (a는 ReferencePath에 대응 필드가 없어 버림). n=0(계획 없음) 또는
// 크기가 안 맞는 방어적 상황은 clear()로 처리한다.
void CBExternalTrajectory(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (!msg) return;

    std::lock_guard<std::mutex> lk(external_traj_mutex);
    external_traj_stamp = ros::Time::now();   // age 보정용 - 데이터 유무와 무관하게 항상 갱신

    if (msg->data.empty()) {
        external_traj.clear();
        return;
    }

    const size_t n = static_cast<size_t>(msg->data[0]);
    if (n == 0 || msg->data.size() < 1 + 6 * n) {
        if (n != 0) {
            ROS_WARN_THROTTLE(1.0, "[MPC] external trajectory size mismatch: n=%zu data.size=%zu",
                               n, msg->data.size());
        }
        external_traj.clear();
        return;
    }

    external_traj.x.resize(n);
    external_traj.y.resize(n);
    external_traj.yaw.resize(n);
    external_traj.k.resize(n);
    external_traj.v.resize(n);

    size_t off = 1;
    for (size_t i = 0; i < n; ++i) external_traj.x[i]   = msg->data[off + i];
    off += n;
    for (size_t i = 0; i < n; ++i) external_traj.y[i]   = msg->data[off + i];
    off += n;
    for (size_t i = 0; i < n; ++i) external_traj.yaw[i] = msg->data[off + i];
    off += n;
    for (size_t i = 0; i < n; ++i) external_traj.k[i]   = msg->data[off + i];
    off += n;
    for (size_t i = 0; i < n; ++i) external_traj.v[i]   = msg->data[off + i];
    // off += n 이후 a[n] 구간은 의도적으로 안 읽음
}

// ========================================
// CtrlCmd 발행 (변경 없음)
// ========================================
void publishCtrlCmd(double steering_rad,
                    double accel_norm,
                    double brake_norm)
{
    morai_msgs::CtrlCmd cmd;
    cmd.longlCmdType = 1;
    cmd.steering  = steering_rad;
    cmd.accel        = clip(accel_norm, 0.0, 1.0);
    cmd.brake        = clip(brake_norm, 0.0, 1.0);
    cmd_pub.publish(cmd);
}

// ========================================
// 메인 제어 루프
//   역할: Controller만
//   Planner(buildReferenceFromWaypoints)는 path_planner.cpp 에서 담당
// ========================================
void controlLoop(const ros::TimerEvent&)
{
    // 1) ego 스냅샷
    MPCState ego_snap;
    {
        std::lock_guard<std::mutex> lk(ego_mutex);
        ego_snap = ego;
    }

    if (!ego_received) {
        ROS_WARN_THROTTLE(1.0, "[MPC] Waiting for /Ego_topic...");
        publishCtrlCmd(0.0, 0.0, 0.3);
        return;
    }

    // 2) Planner(frenet_planner_node) 외부 궤적 스냅샷 - v5: 유일한 주행 소스
    //    (CSV buildReferenceFromWaypoints()는 controlLoop에서 더 이상 호출하지
    //    않는다. 코드는 재사용 가능성 대비 남겨둠 - path_planner.hpp 참고)
    ExternalTrajectory ext_snap;
    ros::Time ext_stamp_snap;
    {
        std::lock_guard<std::mutex> lk(external_traj_mutex);
        ext_snap = external_traj;
        ext_stamp_snap = external_traj_stamp;
    }

    ReferencePath ref;
    std::string active_source;   // EXTERNAL / EXTERNAL_GRACE / STOPPED
    static int consecutive_empty_count = 0;
    const ros::Time now = ros::Time::now();

    if (ext_snap.valid()) {
        ReferencePath built;
        if (buildReferenceFromExternalTrajectory(ext_snap, mpc_params, built)) {
            // ref[0]이 "지금"을 나타내려면, planner가 이 궤적을 만든 시점(수신
            // 시각으로 근사) 이후 실제로 흐른 시간만큼 인덱스를 밀어야 한다 -
            // 안 하면 ref[0]과 실제 ego가 (경과시간 x 속도)만큼 계속 어긋난다
            // (MORAI 실측으로 확인, 2026-07-10). global.hpp의 time_offset_steps
            // 설계 노트 참고.
            const double age = (now - ext_stamp_snap).toSec();
            built.time_offset_steps = static_cast<int>(std::lround(age / mpc_params.dt));
            ref = built;
            external_ref_last_good = built;
            external_ref_last_good_stamp = ext_stamp_snap;
            active_source = "EXTERNAL";
            consecutive_empty_count = 0;
        }
    }

    if (active_source.empty()) {   // 이번 사이클 n==0 (또는 변환 실패)
        consecutive_empty_count++;
        const bool have_grace = !external_ref_last_good.empty() &&
            (now - external_ref_last_good_stamp).toSec() < mpc_params.external_empty_grace_s;

        if (have_grace) {
            ref = external_ref_last_good;
            // grace 동안 계속 흐른 시간만큼 매 사이클 다시 계산 - 정지된 값이 아니라
            // 매번 "지금 기준 몇 스텝째인지"로 갱신해야 EXTERNAL_GRACE가 길어질수록
            // 어긋남이 누적되는 걸 막을 수 있다.
            const double age = (now - external_ref_last_good_stamp).toSec();
            ref.time_offset_steps = static_cast<int>(std::lround(age / mpc_params.dt));
            active_source = "EXTERNAL_GRACE";
            ROS_WARN_THROTTLE(1.0, "[MPC] frenet_planner empty this cycle (%d consecutive) - "
                                    "coasting on last valid trajectory (grace %.2fs)",
                               consecutive_empty_count, mpc_params.external_empty_grace_s);
        } else {
            ROS_WARN("[MPC] frenet_planner empty beyond grace(%.2fs, %d consecutive) - STOPPING "
                     "(no CSV fallback by design)", mpc_params.external_empty_grace_s, consecutive_empty_count);
            publishCtrlCmd(0.0, 0.0, 0.5);
            return;
        }
    }

    ROS_INFO_THROTTLE(2.0, "[MPC] active reference source: %s", active_source.c_str());

    // 3) MPC 풀기 (Controller 역할만)
    MPCResult res = solveMPC(ego_snap, ref,
                             last_control, g_warm_start, mpc_params);

    if (!res.success) {
        ROS_WARN_THROTTLE(1.0, "[MPC] solver msg: %s", res.solver_msg.c_str());
    }

    // 5) warm-start 갱신
    if (res.success && res.controls.size() == static_cast<size_t>(mpc_params.horizon)) {
        g_warm_start = res.controls;
        last_control = res.control;
    } else {
        ROS_WARN("[MPC WS KEEP] keep previous warm_start. success=%d controls=%zu horizon=%d",
            res.success, res.controls.size(), mpc_params.horizon);
    }

    // 6) 제어 출력 변환 및 발행
    double steer = clip(res.control.delta,
                        -mpc_params.steering_max,
                         mpc_params.steering_max);

    double accel_raw  = res.control.accel;
    double accel_norm = 0.0, brake_norm = 0.0;
    // accel_raw가 데드밴드 안이면 액셀/브레이크 둘 다 0(관성 주행) - 그 경계를
    // 그냥 넘나들면 미세한 값 차이로도 완전히 다른 액추에이터(액셀<->브레이크)로
    // 갈아타서 깔짝거리게 됨 (global.hpp의 accel_deadband 설계 노트 참고).
    if (accel_raw > mpc_params.accel_deadband) {
        accel_norm = clip(accel_raw / mpc_params.accel_max, 0.0, 1.0);
    } else if (accel_raw < -mpc_params.accel_deadband) {
        brake_norm = clip(-accel_raw / std::fabs(mpc_params.accel_min), 0.0, 1.0);
    }
    publishCtrlCmd(steer, accel_norm, brake_norm);

    ROS_INFO_THROTTLE(1.0,
        "[MPC] pos=(%.2f,%.2f) yaw=%.2f vx=%.2f | "
        "steer=%.3f rad | accel_raw=%.3f m/s2 | accel=%.2f brake=%.2f | cost=%.2f",
        ego_snap.x, ego_snap.y, ego_snap.yaw, ego_snap.vx,
        steer, accel_raw, accel_norm, brake_norm, res.cost);
}