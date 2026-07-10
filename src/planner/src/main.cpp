#include <mutex>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/ObjectStatusList.h>
#include <behavior_planner/BehaviorContext.h>

#include "global/global.hpp"
#include "global/data_logger.hpp"
#include "global/behavior_bridge.hpp"
#include "frenet/ref_line.hpp"
#include "frenet/path_generator.hpp"
#include "frenet/cost.hpp"
#include "frenet/collision_checker.hpp"
#include "math/frenet_converter.hpp"
#include "visualization/visualization.hpp"

// ---- 전역 상태 (mpc_controller/node/mpc_node.cpp와 동일한 컨벤션) ----

std::mutex g_ego_mutex;
CartesianState g_ego_cs{0, 0, 0, 0, 0, 0};
bool g_ego_received = false;   // /Ego_topic 최초 수신 여부

std::mutex g_obstacle_mutex;
std::vector<ObjectInfo> g_obstacles;

std::mutex g_behavior_mutex;
behavior_planner::BehaviorContext g_behavior_ctx;
bool g_behavior_received = false;
ros::Time g_behavior_stamp;

RefLine g_ref;
bool g_ref_loaded = false;

PathGeneratorConfig   g_path_cfg{};
KinematicLimits       g_limits{};
CostWeights           g_cost_weights{};
VehicleShape          g_vehicle_shape{};
CollisionCheckConfig  g_collision_cfg{};
BehaviorBridgeConfig  g_bridge_cfg{};
double g_target_speed = 8.0;   // BehaviorContext 미수신/stale일 때 쓰는 고정 목표속도 [m/s]

// 경로 연속성(히스테리시스) - g_speed_ref와 같은 컨벤션으로 사이클 간 상태를 들고 다님.
HysteresisConfig g_hysteresis_cfg{};
LastBestParams   g_last_best{};

ros::Publisher g_traj_pub;
ros::Publisher g_marker_pub;
ros::Publisher g_feedback_pub;
std::string g_viz_frame_id = "map";       // rviz Fixed Frame과 일치해야 함 (params.yaml에서 로드)
std::string g_ego_frame_id = "ego_vehicle";  // rviz 카메라가 따라갈 tf 프레임 이름
double g_wheelbase = 3.0;  // [m] 축거. CBEgoState의 kappa 추정(자전거 모델)에 사용 (mpc_controller/mpc_params.yaml과 동일).

// desired_speed로 즉시 스냅하지 않고 comfort_longitudinal_accel로 서서히 수렴시키는
// 레퍼런스 속도 (behavior_bridge.hpp의 BehaviorBridgeConfig::comfort_longitudinal_accel
// 설계 노트 참고). 최초값은 미수신 상태를 나타내는 음수로 둬서 첫 사이클에 실제
// ego 속도로 즉시 맞춰지게 한다 (0.0으로 두면 첫 사이클에 이미 달리고 있던 차량이
// "0에서 서서히 가속"하는 것처럼 오판된다).
double g_speed_ref = -1.0;

double MoraiHeadingToYawRad(double heading_deg) {
    return heading_deg * M_PI / 180.0;
}

// morai_msgs::ObjectStatus -> ObjectInfo (global.hpp) 변환
ObjectInfo ConvertObjectStatus(const morai_msgs::ObjectStatus& obj) {
    ObjectInfo info;
    info.id = obj.unique_id;
    info.type = obj.type;
    info.x = obj.position.x;
    info.y = obj.position.y;
    info.heading = MoraiHeadingToYawRad(obj.heading);
    info.speed = std::hypot(obj.velocity.x, obj.velocity.y);
    info.length = obj.size.x;
    info.width  = obj.size.z;
    return info;
}

void CBEgoState(const morai_msgs::EgoVehicleStatus::ConstPtr& msg) {
    if (!msg) return;

    std::lock_guard<std::mutex> lk(g_ego_mutex);
    g_ego_cs.x = msg->position.x;
    g_ego_cs.y = msg->position.y;
    g_ego_cs.yaw = MoraiHeadingToYawRad(msg->heading);
    g_ego_cs.v = msg->velocity.x;
    g_ego_cs.a = msg->acceleration.x;
    // 자전거 모델: kappa = tan(wheel_angle) / wheelbase.
    g_ego_cs.kappa = std::tan(msg->wheel_angle * M_PI / 180.0) / g_wheelbase;

    g_ego_received = true;
}

// =========================================================
// 콜백: 장애물 목록
// =========================================================
void CBObjects(const morai_msgs::ObjectStatusList::ConstPtr& msg) {
    if (!msg) return;

    std::vector<ObjectInfo> obstacles;
    obstacles.reserve(msg->npc_list.size() + msg->pedestrian_list.size() + msg->obstacle_list.size());
    for (const auto& o : msg->npc_list)        obstacles.push_back(ConvertObjectStatus(o));
    for (const auto& o : msg->pedestrian_list) obstacles.push_back(ConvertObjectStatus(o));
    for (const auto& o : msg->obstacle_list)   obstacles.push_back(ConvertObjectStatus(o));

    std::lock_guard<std::mutex> lk(g_obstacle_mutex);
    g_obstacles = std::move(obstacles);
}

// =========================================================
// 콜백: BehaviorContext (INTEGRATION_PLAN.md 1번)
// =========================================================
void CBBehaviorContext(const behavior_planner::BehaviorContext::ConstPtr& msg) {
    if (!msg) return;

    std::lock_guard<std::mutex> lk(g_behavior_mutex);
    g_behavior_ctx = *msg;
    g_behavior_received = true;
    g_behavior_stamp = ros::Time::now();
}

void PublishCartesianPath(const CartesianPath& cp) {
    std_msgs::Float32MultiArray msg;
    const size_t n = cp.x.size();
    msg.data.reserve(1 + 6 * n);
    msg.data.push_back(static_cast<float>(n));
    for (double v : cp.x)     msg.data.push_back(static_cast<float>(v));
    for (double v : cp.y)     msg.data.push_back(static_cast<float>(v));
    for (double v : cp.yaw)   msg.data.push_back(static_cast<float>(v));
    for (double v : cp.kappa) msg.data.push_back(static_cast<float>(v));
    for (double v : cp.v)     msg.data.push_back(static_cast<float>(v));
    for (double v : cp.a)     msg.data.push_back(static_cast<float>(v));
    g_traj_pub.publish(msg);
}

void PlanningLoop(const ros::TimerEvent&) {
    if (!g_ref_loaded) {
        ROS_WARN_THROTTLE(1.0, "[FrenetPlanner] Waiting for reference line (waypoints)...");
        return;
    }
    if (!g_ego_received) {
        ROS_WARN_THROTTLE(1.0, "[FrenetPlanner] Waiting for /Ego_topic...");
        return;
    }

    CartesianState ego_snap;
    {
        std::lock_guard<std::mutex> lk(g_ego_mutex);
        ego_snap = g_ego_cs;
    }
    std::vector<ObjectInfo> obstacles_snap;
    {
        std::lock_guard<std::mutex> lk(g_obstacle_mutex);
        obstacles_snap = g_obstacles;
    }

    // 1. Cartesian -> Frenet 투영 (CartesianToFrenet은 고속 모드 전용,
    //    math/frenet_converter.hpp 헤더 주석 참고)
    double s, s_dot, s_ddot, d, d_prime, d_pprime;
    CartesianToFrenet(g_ref, ego_snap, s, s_dot, s_ddot, d, d_prime, d_pprime);

    // path_generator/cost.hpp의 고속 d(t) 후보 생성은 시간 미분(d_dot,d_ddot)을
    // 쓰므로 어댑터로 변환해준다. 단, 저속 d(s) 후보 생성(Sec.IV-B)은 원본
    // d_prime/d_pprime을 직접 써야 하므로 FrenetState에 같이 보존한다.
    double d_dot, d_ddot;
    ArcDerivToTimeDeriv(s_dot, s_ddot, d_prime, d_pprime, d_dot, d_ddot);

    FrenetState start{s, s_dot, s_ddot, d, d_dot, d_ddot, d_prime, d_pprime};

    // 2. BehaviorContext -> PlannerCommand (staleness면 LANE_KEEPING 폴백,
    //    INTEGRATION_PLAN.md 1.5)
    behavior_planner::BehaviorContext ctx_snapshot;
    bool ctx_fresh = false;
    {
        std::lock_guard<std::mutex> lk(g_behavior_mutex);
        ctx_snapshot = g_behavior_ctx;
        ctx_fresh = g_behavior_received &&
                    (ros::Time::now() - g_behavior_stamp).toSec() < g_bridge_cfg.context_timeout;
    }

    PlannerCommand cmd{};
    if (ctx_fresh) {
        cmd = BuildCommandFromContext(ctx_snapshot, start, g_bridge_cfg, obstacles_snap, g_ref);
    } else {
        ROS_WARN_THROTTLE(1.0, "[FrenetPlanner] No fresh /behavior/context, falling back to LANE_KEEPING");
        cmd.mode = LANE_KEEPING;
        cmd.target_speed = g_target_speed;
    }

    if (cmd.mode == EMERGENCY) {
        ROS_WARN("[FrenetPlanner] EMERGENCY: ego.s=%.2f v=%.2f stop_position=%.2f "
                 "(stop_distance=%.2f) sentinel=%d",
                 start.s, ego_snap.v, cmd.stop_position, cmd.stop_position - start.s,
                 ctx_fresh && ctx_snapshot.stop_before_s >= 1e8);
    }

    // 2.5. desired_speed(cmd.target_speed)로 즉시 스냅하지 않고 comfort_longitudinal_accel로
    // 서서히 수렴시킨다 (behavior_bridge.hpp 설계 노트 참고). target_speed를 실제로
    // 쓰는 모드(velocity-keeping 계열)에서만 램프하고, 그 외(STOP/EMERGENCY/FOLLOWING)는
    // target_speed가 후보 생성에 안 쓰이므로 실제 ego 속도로 동기화해둬서 나중에
    // velocity-keeping으로 복귀할 때 오래된 레퍼런스로 점프하지 않게 한다.
    const bool uses_target_speed = (cmd.mode == LANE_KEEPING || cmd.mode == LANE_CHANGE_LEFT ||
                                     cmd.mode == LANE_CHANGE_RIGHT || cmd.mode == TURN_LEFT ||
                                     cmd.mode == TURN_RIGHT || cmd.mode == AVOID);

    static ros::Time s_last_ramp_time(0);
    const ros::Time now = ros::Time::now();
    double ramp_dt = (s_last_ramp_time.isZero()) ? 0.0 : (now - s_last_ramp_time).toSec();
    ramp_dt = std::max(0.0, std::min(ramp_dt, 0.5));  // 최초 호출/지연 대비 방어적 클램프
    s_last_ramp_time = now;

    if (g_speed_ref < 0.0) g_speed_ref = ego_snap.v;  // 최초 수신: 실제 속도로 즉시 동기화

    if (uses_target_speed) {
        const double max_dv = g_bridge_cfg.comfort_longitudinal_accel * ramp_dt;
        g_speed_ref = std::min(std::max(cmd.target_speed, g_speed_ref - max_dv), g_speed_ref + max_dv);
        cmd.target_speed = g_speed_ref;
    } else {
        g_speed_ref = ego_snap.v;
    }

    // 3. 후보 생성 -> 충돌 필터 -> 비용 평가 -> 최적 선택
    //    EMERGENCY는 승차감보다 최단 정지를 우선하도록 전용 가중치 사용
    //    (INTEGRATION_PLAN.md 1.1.2, (a) 채택).
    const CostWeights& cost_weights = (cmd.mode == EMERGENCY) ? g_bridge_cfg.emergency_cost_weights
                                                                : g_cost_weights;

    std::vector<FrenetPath> candidates =
        ResolveManeuver(start, cmd, g_ref, g_path_cfg, g_limits, g_bridge_cfg.lane_width);

    FilterByCollision(candidates, g_ref, obstacles_snap, g_vehicle_shape, g_collision_cfg);

    EvaluateCosts(candidates, cost_weights);

    const FrenetPath* best =
        SelectBestPathWithHysteresis(candidates, cmd.mode, g_path_cfg, g_hysteresis_cfg, g_last_best);

    // PlanFeedback은 best 유무와 무관하게 매 사이클 발행 (INTEGRATION_PLAN.md 2번).
    g_feedback_pub.publish(BuildFeedback(cmd, best, ctx_snapshot));

    // rviz 시각화는 best 유무와 무관하게 매 사이클 발행한다 — 실패한 순간
    // (후보가 전부 빨갛게 무효화되는 것)도 그대로 눈으로 볼 수 있어야 디버깅에 쓸모있다.
    visualization_msgs::MarkerArray markers;
    for (auto& m : BuildCandidateMarkers(candidates, g_ref, best, g_viz_frame_id).markers)
        markers.markers.push_back(std::move(m));
    markers.markers.push_back(BuildRefLineMarker(g_ref, start.s, 40.0, g_viz_frame_id, 0));
    markers.markers.push_back(BuildEgoMarker(ego_snap, g_vehicle_shape, g_viz_frame_id, 0));
    BroadcastEgoTransform(ego_snap, g_viz_frame_id, g_ego_frame_id);
    for (auto& m : BuildObstacleMarkers(obstacles_snap, g_viz_frame_id).markers)
        markers.markers.push_back(std::move(m));
    g_marker_pub.publish(markers);

    if (!best) {
        ROS_WARN_THROTTLE(1.0, "[FrenetPlanner] No valid candidate this cycle (%zu generated) | "
                                "v=%.2f speed_ref=%.2f target=%.2f",
                           candidates.size(), ego_snap.v, g_speed_ref, cmd.target_speed);
        // mpc_controller가 "토픽이 끊긴 것"과 "이번 사이클에 계획이 없는 것"을
        // 구분할 수 있도록, 빈 CartesianPath(n=0)도 명시적으로 발행한다.
        PublishCartesianPath(CartesianPath{});
        return;
    }

    // 4. Cartesian 변환 + publish
    CartesianPath cp = ConvertToCartesianPath(*best, g_ref);
    PublishCartesianPath(cp);

    ROS_INFO_THROTTLE(1.0,
        "[FrenetPlanner] s=%.2f d=%.2f v=%.2f speed_ref=%.2f target=%.2f | candidates=%zu | "
        "cost_total=%.3f (lat=%.3f lon=%.3f)",
        start.s, start.d, ego_snap.v, g_speed_ref, cmd.target_speed, candidates.size(),
        best->cost_total, best->cost_lat, best->cost_lon);
}

// =========================================================
// main
// =========================================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "frenet_planner_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ROS_INFO("========================================");
    ROS_INFO("  Frenet Frame Planner Node Starting");
    ROS_INFO("  (FSM 미개발 상태 - 고정 LANE_KEEPING으로 검증)");
    ROS_INFO("========================================");

    LoadParams(pnh, g_path_cfg, g_limits, g_cost_weights, g_vehicle_shape, g_collision_cfg,
               g_target_speed, g_bridge_cfg, g_wheelbase, g_hysteresis_cfg);
    // EMERGENCY 정지거리 계산(behavior_bridge.cpp)이 kinematic_limits와 항상 일치하도록,
    // 별도 yaml 키 없이 이미 로드된 값을 그대로 채워 넣는다.
    g_bridge_cfg.emergency_max_decel = g_limits.max_longitudinal_accel;
    pnh.param<std::string>("planner/viz_frame_id", g_viz_frame_id, "map");
    pnh.param<std::string>("planner/ego_frame_id", g_ego_frame_id, "ego_vehicle");

    ros::Publisher global_path_pub =
        nh.advertise<visualization_msgs::MarkerArray>("/frenet_planner/global_path", 1, /*latch=*/true);

    std::string waypoint_file;
    pnh.param<std::string>("waypoint_file", waypoint_file, "");
    if (waypoint_file.empty() || !LoadReferenceLine(waypoint_file, g_ref, g_limits.max_curvature)) {
        ROS_ERROR("[FrenetPlanner] Reference line load failed. Node will keep running "
                   "but will not publish until a valid waypoint_file param is set.");
    } else {
        g_ref_loaded = true;
        visualization_msgs::MarkerArray global_path_markers;
        global_path_markers.markers.push_back(BuildGlobalPathMarker(g_ref, g_viz_frame_id, 0));
        global_path_pub.publish(global_path_markers);
    }

    ros::Subscriber ego_sub = nh.subscribe("/Ego_topic", 1, CBEgoState);
    ros::Subscriber obj_sub = nh.subscribe("/Object_topic", 1, CBObjects);
    ros::Subscriber behavior_sub = nh.subscribe("/behavior/context", 1, CBBehaviorContext);

    g_traj_pub = nh.advertise<std_msgs::Float32MultiArray>("/frenet_planner/trajectory", 1);
    g_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/frenet_planner/markers", 1);
    g_feedback_pub = nh.advertise<behavior_planner::PlanFeedback>("/planner/plan_feedback", 1);

    double planning_hz = 10.0;  // 논문 Sec.VIII: 100ms 고정 주기
    pnh.param<double>("planner/planning_frequency", planning_hz, planning_hz);
    ros::Timer timer = nh.createTimer(ros::Duration(1.0 / planning_hz), PlanningLoop);

    ROS_INFO("[FrenetPlanner] Subscribed: /Ego_topic /Object_topic /behavior/context");
    ROS_INFO("[FrenetPlanner] Publishing: /frenet_planner/trajectory /planner/plan_feedback @ %.1f Hz",
             planning_hz);
    ROS_INFO("========================================");

    ros::spin();
    return 0;
}
