#include "mpc_node.hpp"
#include "../global/utils.hpp"

// ========================================
// 솔버 warm-start 보존용 (이 파일에 한정)
// ========================================
static std::vector<MPCControl> g_warm_start;

// ========================================
// ROS 콜백
// ========================================

void CBGps(const morai_msgs::GPSMessage::ConstPtr& msg)
{
    if (!msg) return;

    // status==0 → GPS jamming (PlanningControl 과 동일한 규칙)
    if (msg->status == 0) {
        gps_jamming_perception = true;
        return;
    }
    gps_jamming_perception = false;

    // ref 미초기화면 첫 GPS 로 자동 초기화
    if (!coord_ref_initialized) {
        coord_ref.lat0 = msg->latitude;
        coord_ref.lon0 = msg->longitude;
        coord_ref.h0   = msg->altitude;
        wgs84ToECEF(coord_ref.lat0, coord_ref.lon0, coord_ref.h0,
                    coord_ref.x0_ecef, coord_ref.y0_ecef, coord_ref.z0_ecef);
        coord_ref_initialized = true;
        ROS_INFO("[MPC] GPS ref auto-init: lat=%.8f lon=%.8f", coord_ref.lat0, coord_ref.lon0);
    }

    double x, y, z;
    wgs84ToENU(msg->latitude, msg->longitude, msg->altitude, coord_ref, x, y, z);

    std::lock_guard<std::mutex> lk(ego_mutex);
    ego.x = x;
    ego.y = y;
}

void CBImu(const sensor_msgs::Imu::ConstPtr& msg)
{
    if (!msg) return;
    double yaw = quaternionToYaw(msg->orientation.x,
                                 msg->orientation.y,
                                 msg->orientation.z,
                                 msg->orientation.w);
    std::lock_guard<std::mutex> lk(ego_mutex);
    ego.yaw = yaw;
}

void CBEgoState(const morai_msgs::EgoVehicleStatus::ConstPtr& msg)
{
    if (!msg) return;
    // PlanningControl 과 동일: 속도만 사용 (yaw 는 IMU 가 권위)
    std::lock_guard<std::mutex> lk(ego_mutex);
    ego.vx = msg->velocity.x;
}

void CBCostmap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    if (!msg) return;
    std::lock_guard<std::mutex> lk(costmap_mutex);
    costmap_info.msg        = msg;          // 원본 보관 (0..100 그대로)
    costmap_info.origin_x   = msg->info.origin.position.x;
    costmap_info.origin_y   = msg->info.origin.position.y;
    costmap_info.resolution = msg->info.resolution;
    costmap_info.width      = static_cast<int>(msg->info.width);
    costmap_info.height     = static_cast<int>(msg->info.height);
    new_costmap_received = true;
}

void CBLanePath(const std_msgs::Float32MultiArray::ConstPtr& /*msg*/)
{
    // 현재 구현에서는 lane perception 을 직접 사용하지 않음.
    // (CSV 기반 waypoints 가 권위. jamming 대응 확장이 필요할 때 활용 가능)
}

// ========================================
// Reference path 빌더
//   현재 ego 위치에서 가장 가까운 waypoint 부터 ref_window 개를 선택
// ========================================
void buildReferenceFromWaypoints()
{
    reference_path.clear();
    if (waypoints.empty()) return;

    // ego 스냅샷
    MPCState ego_snap;
    {
        std::lock_guard<std::mutex> lk(ego_mutex);
        ego_snap = ego;
    }

    // 캐시된 인덱스 부근에서 우선 탐색, 너무 멀면 전체 탐색
    int n = static_cast<int>(waypoints.size());
    int idx = clip(closest_waypoint_idx, 0, n - 1);

    auto sqDist = [&](int i)->double {
        double dx = waypoints[i].x - ego_snap.x;
        double dy = waypoints[i].y - ego_snap.y;
        return dx*dx + dy*dy;
    };

    // sanity: 캐시가 너무 멀면 (>15 m) 전체 탐색
    bool global_search = false;
    if (std::sqrt(sqDist(idx)) > 15.0) global_search = true;

    int start, end;
    if (global_search) {
        start = 0;
        end = n;
    } else {
        start = std::max(0, idx - 10);
        end   = std::min(n, idx + 50);
    }
    int best = idx;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (int i = start; i < end; ++i) {
        double d2 = sqDist(i);
        if (d2 < best_d2) { best_d2 = d2; best = i; }
    }
    closest_waypoint_idx = best;

    // window: best 부터 ref_window 개 (앞쪽으로만)
    int W = mpc_params.ref_window;
    int last = std::min(n, best + W);

    reference_path.x_ref.reserve(last - best);
    reference_path.y_ref.reserve(last - best);
    reference_path.yaw_ref.reserve(last - best);
    reference_path.v_ref.reserve(last - best);
    reference_path.k_ref.reserve(last - best);

    // 곡률 기반 target velocity 결정 (PlanningControl 의 다단계 로직 단순화)
    auto velocityFromCurvature = [&](double k)->double {
        if (k > mpc_params.curve_th_sharp) return mpc_params.curve_vel_sharp;
        if (k > mpc_params.curve_th_mid)   return mpc_params.curve_vel_mid;
        if (k > mpc_params.curve_th_mild)  return mpc_params.curve_vel_mild;
        return mpc_params.target_vel;
    };

    for (int i = best; i < last; ++i) {
        reference_path.x_ref.push_back(waypoints[i].x);
        reference_path.y_ref.push_back(waypoints[i].y);
        reference_path.k_ref.push_back(waypoints[i].curvature);

        // yaw 는 인접 점의 방향
        double yaw_ref = 0.0;
        if (i + 1 < n) {
            yaw_ref = std::atan2(waypoints[i+1].y - waypoints[i].y,
                                  waypoints[i+1].x - waypoints[i].x);
        } else if (i > 0) {
            yaw_ref = std::atan2(waypoints[i].y - waypoints[i-1].y,
                                  waypoints[i].x - waypoints[i-1].x);
        }
        reference_path.yaw_ref.push_back(yaw_ref);
        reference_path.v_ref.push_back(velocityFromCurvature(waypoints[i].curvature));
    }
    // v_ref smoothing: 계단식 속도 변화를 완화
    if (reference_path.v_ref.size() >= 3) {
        std::vector<double> smoothed = reference_path.v_ref;

        int smooth_radius = 30;  // waypoint 간격 0.1m면 앞뒤 약 0.5m
        for (size_t i = 0; i < reference_path.v_ref.size(); ++i) {
            size_t start = (i > static_cast<size_t>(smooth_radius)) ? i - smooth_radius : 0;
            size_t end = std::min(reference_path.v_ref.size() - 1, i + static_cast<size_t>(smooth_radius));

            double sum = 0.0;
            int count = 0;
            for (size_t j = start; j <= end; ++j) {
                sum += reference_path.v_ref[j];
                ++count;
            }
            smoothed[i] = sum / count;
        }

        reference_path.v_ref = smoothed;
    }
    new_reference_path_received = true;
}

// ========================================
// 종방향 PID
//   target_vel - current_vel  →  normalized [0..1] accel/brake
// ========================================
void velocityPID(double v_target, double v_current,
                 double& out_accel_norm, double& out_brake_norm)
{
    static double prev_error = 0.0;
    static double integral   = 0.0;
    static ros::Time last_t  = ros::Time::now();

    ros::Time now = ros::Time::now();
    double dt = (now - last_t).toSec();
    last_t = now;
    if (dt <= 0.0 || dt > 1.0) dt = 1.0 / std::max(1.0, mpc_params.control_frequency);

    double e = v_target - v_current;
    integral += e * dt;
    integral = clip(integral, -10.0, 10.0);
    double d = (e - prev_error) / dt;
    prev_error = e;

    double u = mpc_params.pid_kp * e + mpc_params.pid_ki * integral + mpc_params.pid_kd * d;

    if (u >= 0.0) {
        out_accel_norm = std::min(u, 1.0);
        out_brake_norm = 0.0;
    } else {
        out_accel_norm = 0.0;
        out_brake_norm = std::min(-u, 1.0);
    }
}

// ========================================
// CtrlCmd 발행 (MORAI accel/brake 모드 = longlCmdType 1)
// ========================================
void publishCtrlCmd(double steering_rad,
                    double accel_norm,
                    double brake_norm)
{
    morai_msgs::CtrlCmd cmd;
    cmd.longlCmdType = 1;                              // accel/brake mode
    cmd.front_steer     = -steering_rad;                   // [rad] 부호 반전
    cmd.accel        = clip(accel_norm, 0.0, 1.0);
    cmd.brake        = clip(brake_norm, 0.0, 1.0);
    cmd_pub.publish(cmd);
}

// ========================================
// 메인 제어 루프
// ========================================
void controlLoop(const ros::TimerEvent&)
{
    // 1) ego 스냅샷
    MPCState ego_snap;
    {
        std::lock_guard<std::mutex> lk(ego_mutex);
        ego_snap = ego;
    }

    // 좌표 미초기화면 안전: 정지 명령
    if (!coord_ref_initialized) {
        ROS_WARN_THROTTLE(1.0, "[MPC] Waiting for GPS reference initialization...");
        publishCtrlCmd(0.0, 0.0, 0.3);
        return;
    }

    // 2) reference path 빌드 (현재 위치 주변)
    buildReferenceFromWaypoints();
    if (reference_path.empty()) {
        ROS_WARN_THROTTLE(1.0, "[MPC] Reference path empty");
        publishCtrlCmd(0.0, 0.0, 0.5);
        return;
    }

    // 3) costmap 스냅샷
    CostmapInfo costmap_snap;
    {
        std::lock_guard<std::mutex> lk(costmap_mutex);
        costmap_snap = costmap_info;
    }

    // 4) MPC 해 구하기
    MPCResult res = solveMPC(ego_snap, reference_path, costmap_snap,
                              last_control, g_warm_start, mpc_params);

    if (!res.success) {
        ROS_WARN_THROTTLE(1.0, "[MPC] solver msg: %s", res.solver_msg.c_str());
    }

    // 5) warm-start 갱신
    //g_warm_start = res.controls;
    //last_control = res.control;
    if (res.success && res.controls.size() == static_cast<size_t>(mpc_params.horizon)) {
    g_warm_start = res.controls;
    last_control = res.control;
    } else {
        ROS_WARN("[MPC WS KEEP] keep previous warm_start. success=%d controls=%zu horizon=%d",
            res.success, res.controls.size(), mpc_params.horizon);
}

    // 6) 종방향: PID 가 preview 기반 target 을 추종
    double v_target = mpc_params.target_vel;
    double debug_min_v = v_target;
    double debug_dist_to_min_v = 0.0;
    double debug_v_allowed = v_target;

    if (!reference_path.v_ref.empty()) {
        // 기본은 현재 위치의 reference 속도
        v_target = reference_path.v_ref.front();

        // 앞쪽 preview 거리 설정
        const double preview_distance = 70.0;  // [m], 도심 90도 코너면 15~25m 후보
        const double comfortable_decel = 0.7;  // [m/s^2]

        double accum_dist = 0.0;
        double min_v = reference_path.v_ref.front();
        double dist_to_min_v = 0.0;

        for (size_t i = 1; i < reference_path.v_ref.size(); ++i) {
            double dx = reference_path.x_ref[i] - reference_path.x_ref[i - 1];
            double dy = reference_path.y_ref[i] - reference_path.y_ref[i - 1];
            accum_dist += std::hypot(dx, dy);

            if (accum_dist > preview_distance) {
                break;
            }

            if (reference_path.v_ref[i] < min_v) {
                min_v = reference_path.v_ref[i];
                dist_to_min_v = accum_dist;
            }
        }

        // 코너 속도 min_v까지 남은 거리 dist_to_min_v를 고려한 허용 현재속도
        double v_allowed = std::sqrt(min_v * min_v + 2.0 * comfortable_decel * dist_to_min_v);

        // 너무 높은 목표를 막고, 코너 접근 시 자연스럽게 낮춤
        v_target = std::min(v_target, v_allowed);
        v_target = std::min(v_target, mpc_params.target_vel);

        // 안전 하한. 너무 저속 고정 방지용, 필요 없으면 생략 가능
        v_target = std::max(v_target, 2.0);
        debug_min_v = min_v;
        debug_dist_to_min_v = dist_to_min_v;
        debug_v_allowed = v_allowed;
    }
    double accel_norm, brake_norm;
    velocityPID(v_target, ego_snap.vx, accel_norm, brake_norm);

    // 7) 발행 (steering 은 MPC 결과, accel/brake 는 PID 결과)
    double steer = clip(res.control.delta,
                                     -mpc_params.steering_max,
                                      mpc_params.steering_max);
    publishCtrlCmd(steer, accel_norm, brake_norm);

    ROS_INFO_THROTTLE(1.0,
        "[MPC] pos=(%.2f,%.2f) yaw=%.2f vx=%.2f | tgt=%.2f min_v=%.2f dist=%.1f allow=%.2f | steer=%.3f rad | accel=%.2f brake=%.2f | cost=%.2f",
        ego_snap.x, ego_snap.y, ego_snap.yaw, ego_snap.vx,
        v_target, debug_min_v, debug_dist_to_min_v, debug_v_allowed,
        steer, accel_norm, brake_norm, res.cost);
}
