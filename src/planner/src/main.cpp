#include <mutex>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/ObjectStatusList.h>

#include "global/global.hpp"
#include "global/data_logger.hpp"
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

RefLine g_ref;
bool g_ref_loaded = false;

PathGeneratorConfig  g_path_cfg{};
KinematicLimits      g_limits{};
CostWeights          g_cost_weights{};
VehicleShape         g_vehicle_shape{};
CollisionCheckConfig g_collision_cfg{};
double g_target_speed = 8.0;   // FSM 없을 때 쓰는 고정 목표속도 [m/s]

ros::Publisher g_traj_pub;
ros::Publisher g_marker_pub;
std::string g_viz_frame_id = "map";       // rviz Fixed Frame과 일치해야 함 (params.yaml에서 로드)
std::string g_ego_frame_id = "ego_vehicle";  // rviz 카메라가 따라갈 tf 프레임 이름

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
    // TODO(알려진 한계, 실측 확인됨): kappa를 EgoVehicleStatus가 안 줘서 0으로
    // 근사. 급조향 순간 그 사이클의 후보가 전부 무효화됐다가 다음 사이클에
    // 자연 복구되는 현상 있음. 근본 해결: wheel_angle+wheelbase로 kappa 추정.
    g_ego_cs.kappa = 0.0;

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

    // 2. FSM 대체용 고정 PlannerCommand (파일 상단 설명 참고)
    PlannerCommand cmd{};
    cmd.mode = LANE_KEEPING;
    cmd.target_speed = g_target_speed;

    // 3. 후보 생성 -> 충돌 필터 -> 비용 평가 -> 최적 선택
    std::vector<FrenetPath> candidates = ResolveManeuver(start, cmd, g_ref, g_path_cfg, g_limits);
    FilterByCollision(candidates, g_ref, obstacles_snap, g_vehicle_shape, g_collision_cfg);
    EvaluateCosts(candidates, g_cost_weights);

    const FrenetPath* best = SelectBestPath(candidates);

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

    LoadParams(pnh, g_path_cfg, g_limits, g_cost_weights, g_vehicle_shape, g_collision_cfg, g_target_speed);
    pnh.param<std::string>("planner/viz_frame_id", g_viz_frame_id, "map");
    pnh.param<std::string>("planner/ego_frame_id", g_ego_frame_id, "ego_vehicle");

    ros::Publisher global_path_pub =
        nh.advertise<visualization_msgs::MarkerArray>("/frenet_planner/global_path", 1, /*latch=*/true);

    std::string waypoint_file;
    pnh.param<std::string>("waypoint_file", waypoint_file, "");
    if (waypoint_file.empty() || !LoadReferenceLine(waypoint_file, g_ref)) {
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

    g_traj_pub = nh.advertise<std_msgs::Float32MultiArray>("/frenet_planner/trajectory", 1);
    g_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/frenet_planner/markers", 1);

    double planning_hz = 10.0;  // 논문 Sec.VIII: 100ms 고정 주기
    pnh.param<double>("planner/planning_frequency", planning_hz, planning_hz);
    ros::Timer timer = nh.createTimer(ros::Duration(1.0 / planning_hz), PlanningLoop);

    ROS_INFO("[FrenetPlanner] Subscribed: /Ego_topic /Object_topic");
    ROS_INFO("[FrenetPlanner] Publishing: /frenet_planner/trajectory @ %.1f Hz", planning_hz);
    ROS_INFO("========================================");

    ros::spin();
    return 0;
}
