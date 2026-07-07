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

    // 2) Planner 호출: ReferencePath 생성
    //    buildReferenceFromWaypoints() 는 path_planner.cpp 에서 담당
    //    mpc_node 는 결과(ReferencePath)만 받아서 사용
    ReferencePath ref;
    if (!buildReferenceFromWaypoints(ego_snap, waypoints, mpc_params,
                                     ref, closest_waypoint_idx)) {
        ROS_WARN_THROTTLE(1.0, "[MPC] Reference path empty");
        publishCtrlCmd(0.0, 0.0, 0.5);
        return;
    }

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
    if (accel_raw >= 0.0) {
        accel_norm = clip(accel_raw / mpc_params.accel_max, 0.0, 1.0);
    } else {
        brake_norm = clip(-accel_raw / std::fabs(mpc_params.accel_min), 0.0, 1.0);
    }
    publishCtrlCmd(steer, accel_norm, brake_norm);

    ROS_INFO_THROTTLE(1.0,
        "[MPC] pos=(%.2f,%.2f) yaw=%.2f vx=%.2f | "
        "steer=%.3f rad | accel_raw=%.3f m/s2 | accel=%.2f brake=%.2f | cost=%.2f",
        ego_snap.x, ego_snap.y, ego_snap.yaw, ego_snap.vx,
        steer, accel_raw, accel_norm, brake_norm, res.cost);
}