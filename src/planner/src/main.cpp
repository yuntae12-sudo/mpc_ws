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

ros::Publisher g_traj_pub;
ros::Publisher g_marker_pub;
ros::Publisher g_feedback_pub;
std::string g_viz_frame_id = "map";       // rviz Fixed Frame과 일치해야 함 (params.yaml에서 로드)
std::string g_ego_frame_id = "ego_vehicle";  // rviz 카메라가 따라갈 tf 프레임 이름
double g_wheelbase = 3.0;  // [m] 축거. CBEgoState의 kappa 추정(자전거 모델)에 사용 (mpc_controller/mpc_params.yaml과 동일).

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
    // TODO(검증 필요): size가 (length,width,height)인지 (width,length,height)인지
    // 실측 확인 안 됨. 지금은 x=length, y=width로 가정.
    info.length = obj.size.x;
    info.width  = obj.size.y;
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
    // TODO(검증 필요): wheel_angle의 단위(deg 가정, heading과 동일 관례)와
    // 부호(좌회전=양수 가정)가 실측으로 확인되지 않음.
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

    // path_generator/cost.hpp가 기대하는 FrenetState는 시간 미분(d_dot,d_ddot)
    // 이므로, arc-length 미분(d_prime,d_pprime)에서 어댑터로 변환해준다.
    double d_dot, d_ddot;
    ArcDerivToTimeDeriv(s_dot, s_ddot, d_prime, d_pprime, d_dot, d_ddot);

    FrenetState start{s, s_dot, s_ddot, d, d_dot, d_ddot};

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
        cmd = BuildCommandFromContext(ctx_snapshot, start, g_bridge_cfg);
    } else {
        ROS_WARN_THROTTLE(1.0, "[FrenetPlanner] No fresh /behavior/context, falling back to LANE_KEEPING");
        cmd.mode = LANE_KEEPING;
        cmd.target_speed = g_target_speed;
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

    const FrenetPath* best = SelectBestPath(candidates);

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
        ROS_WARN_THROTTLE(1.0, "[FrenetPlanner] No valid candidate this cycle (%zu generated)",
                           candidates.size());
        return;
    }

    // 4. Cartesian 변환 + publish
    CartesianPath cp = ConvertToCartesianPath(*best, g_ref);
    PublishCartesianPath(cp);

    ROS_INFO_THROTTLE(1.0,
        "[FrenetPlanner] s=%.2f d=%.2f v=%.2f | candidates=%zu | cost_total=%.3f (lat=%.3f lon=%.3f)",
        start.s, start.d, ego_snap.v, candidates.size(),
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
               g_target_speed, g_bridge_cfg, g_wheelbase);
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
