// =========================================================
// Frenet Frame Planner 노드 (ROS1 / MORAI 시뮬레이터 연동)
//
// 이 노드의 목적(지금 시점 스코프):
//   FSM(behavioral layer)이 아직 개발되지 않은 상태라, PlannerCommand를
//   FSM 없이 고정값(LANE_KEEPING + 파라미터로 받은 목표속도)으로 채워서
//   Frenet Frame Planner 파이프라인 자체를 MORAI 시뮬레이터 위에서
//   단독으로 검증하기 위한 노드다.
//     CartesianToFrenet(시작상태 투영)
//       -> ResolveManeuver(후보 생성, Sec.IV/V)
//       -> FilterByCollision(Sec.VI)
//       -> EvaluateCosts(Prop.1, Sec.VI)
//       -> SelectBestPath(Sec.VII)
//       -> ConvertToCartesianPath
//       -> publish
//   FSM이 붙으면 PlanningLoop()의 "PlannerCommand 고정값" 부분만 구독
//   콜백 결과로 교체하면 되고, 파이프라인 로직 자체는 그대로 재사용된다.
//
// 자매 패키지 mpc_controller(src/node/mpc_node.cpp)의 관례를 그대로 따름:
//   - EgoVehicleStatus/ObjectStatusList 구독, CSV 기반 waypoints,
//     ros::Timer 고정주기, pnh.param<T>() 파라미터 로드.
//   mpc_controller는 GPS+IMU를 직접 융합해서 ego 상태를 만드는데, 그건
//   실제 센서 노이즈(GPS 재밍 등)를 모델링하려는 목적이고, 이 노드는
//   "Planner 알고리즘 자체 검증"이 목적이라 EgoVehicleStatus의
//   ground-truth 값을 그대로 쓴다. 나중에 실차/노이즈 대응이 필요해지면
//   mpc_controller의 GPS+IMU 융합 방식으로 교체하면 됨.
// =========================================================

#include <mutex>
#include <fstream>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/ObjectStatusList.h>

#include "global/global.hpp"
#include "frenet/ref_line.hpp"
#include "frenet/path_generator.hpp"
#include "frenet/cost.hpp"
#include "frenet/collision_checker.hpp"
#include "math/frenet_converter.hpp"

// ---- 전역 상태 (mpc_controller/node/mpc_node.cpp와 동일한 컨벤션) ----

std::mutex g_ego_mutex;
CartesianState g_ego_cs{0, 0, 0, 0, 0, 0};
bool g_ego_received = false;

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
    // TODO(검증 필요): MORAI ObjectStatus.size(Vector3)가 (length, width, height)
    // 순서인지 (width, length, height) 순서인지 실제 메시지 값으로 확인 필요.
    // 지금은 x=length(진행방향), y=width(횡방향)로 가정.
    info.length = obj.size.x;
    info.width  = obj.size.y;
    return info;
}

// =========================================================
// 콜백: 자차 상태
// =========================================================
void CBEgoState(const morai_msgs::EgoVehicleStatus::ConstPtr& msg) {
    if (!msg) return;

    const double yaw = MoraiHeadingToYawRad(msg->heading);

    std::lock_guard<std::mutex> lk(g_ego_mutex);
    g_ego_cs.x = msg->position.x;
    g_ego_cs.y = msg->position.y;
    g_ego_cs.yaw = yaw;
    // world-frame velocity/acceleration을 heading 방향으로 투영해서
    // 종방향 스칼라 v, a로 사용 (frenet_converter의 CartesianState 정의와 일치)
    g_ego_cs.v = msg->velocity.x * std::cos(yaw) + msg->velocity.y * std::sin(yaw);
    g_ego_cs.a = msg->acceleration.x * std::cos(yaw) + msg->acceleration.y * std::sin(yaw);
    // TODO(추후 개발 필요): 자차 곡률(kappa)은 EgoVehicleStatus가 직접 제공하지
    // 않는다. wheel_angle과 차량 wheelbase를 알면 kappa = tan(wheel_angle)/wheelbase
    // 로 근사 가능하지만, 지금 프로젝트엔 wheelbase 파라미터가 아직 없어 0으로
    // 둔다. 이 값은 CartesianToFrenet의 d''(d_pprime) 계산에만 쓰이고, 매
    // planning cycle마다 그 시점의 실제 상태로 다시 계산되므로 오차가 누적되지
    // 않는다 (재계획 시작점 한 스텝에만 국한된 근사 오차).
    g_ego_cs.kappa = 0.0;

    g_ego_received = true;
}

// =========================================================
// 콜백: 장애물 목록
// ObjectStatusList의 npc_list/pedestrian_list/obstacle_list 구조가
// global.hpp의 FsmInput과 이름까지 동일한 건 우연이 아니라, FsmInput이
// 애초에 이 메시지를 본떠서 설계된 것으로 보인다.
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
// CartesianPath -> std_msgs/Float32MultiArray 인코딩
//
// 레이아웃(고정 순서, 문서화 필수 — Float32MultiArray 자체엔 의미 정보가 없어서
// 구독 측(mpc_controller 등)이 반드시 이 순서를 그대로 알고 파싱해야 함):
//   data[0]              = N (샘플 개수)
//   data[1        .. N]  = x[0..N-1]
//   data[1+N      .. 2N] = y[0..N-1]
//   data[1+2N     .. 3N] = yaw[0..N-1]
//   data[1+3N     .. 4N] = kappa[0..N-1]
//   data[1+4N     .. 5N] = v[0..N-1]
//   data[1+5N     .. 6N] = a[0..N-1]
// =========================================================
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

// =========================================================
// 플래닝 루프
// 논문 Sec.VIII: "planning cycle of 100 ms" (자율주행차 JUNIOR 실험 설정)
// 를 그대로 채용 — main()에서 고정주기 ros::Timer로 이 함수를 호출한다.
// =========================================================
void PlanningLoop(const ros::TimerEvent&) {
    if (!g_ref_loaded) {
        ROS_WARN_THROTTLE(1.0, "[FrenetPlanner] Waiting for reference line (waypoints)...");
        return;
    }
    if (!g_ego_received) {
        ROS_WARN_THROTTLE(1.0, "[FrenetPlanner] Waiting for ego state...");
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
// waypoints CSV -> RefLine 구성
// mpc_controller/global/parameter_loader.cpp의 loadWaypoints()와 동일한
// 포맷("x y" 공백 구분 또는 "x,y" 콤마 구분, 한 줄에 한 점)을 그대로 지원
// (같은 CSV 파일을 두 노드가 공유해서 로드할 수 있게).
// =========================================================
bool LoadReferenceLine(const std::string& path, RefLine& out_ref) {
    std::ifstream file(path);
    if (!file.is_open()) {
        ROS_ERROR("[FrenetPlanner] Failed to open waypoint file: %s", path.c_str());
        return false;
    }

    std::vector<double> wx, wy;
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;

        // 공백 우선
        {
            std::stringstream ss(line);
            double x, y;
            if (ss >> x >> y) {
                wx.push_back(x);
                wy.push_back(y);
                continue;
            }
        }

        // CSV (콤마 구분)
        std::stringstream ss(line);
        std::string val;
        std::vector<double> row;
        while (std::getline(ss, val, ',')) {
            val.erase(0, val.find_first_not_of(" \t\r\n"));
            if (!val.empty()) val.erase(val.find_last_not_of(" \t\r\n") + 1);
            if (val.empty()) continue;
            try { row.push_back(std::stod(val)); } catch (...) { /* 헤더 라인 등 skip */ }
        }
        if (row.size() >= 2) {
            wx.push_back(row[0]);
            wy.push_back(row[1]);
        }
    }

    if (wx.size() < 2) {
        ROS_ERROR("[FrenetPlanner] Waypoint file has fewer than 2 valid points: %s", path.c_str());
        return false;
    }

    out_ref = BuildRefLine(wx, wy);
    ROS_INFO("[FrenetPlanner] Loaded %zu waypoints from %s", wx.size(), path.c_str());
    return true;
}

// =========================================================
// params.yaml -> 각 config struct 로드 (config/params.yaml 키 구조와 동일)
// =========================================================
void LoadParams(ros::NodeHandle& pnh) {
    pnh.param<double>("planner/path_generator/lateral_d1/min",   g_path_cfg.lateral_d1.min,   -1.5);
    pnh.param<double>("planner/path_generator/lateral_d1/max",   g_path_cfg.lateral_d1.max,    1.5);
    pnh.param<double>("planner/path_generator/lateral_d1/step",  g_path_cfg.lateral_d1.step,   0.5);
    pnh.param<double>("planner/path_generator/time_horizon/min", g_path_cfg.time_horizon.min,  2.0);
    pnh.param<double>("planner/path_generator/time_horizon/max", g_path_cfg.time_horizon.max,  5.0);
    pnh.param<double>("planner/path_generator/time_horizon/step",g_path_cfg.time_horizon.step, 1.0);
    pnh.param<double>("planner/path_generator/delta_s/min",      g_path_cfg.delta_s.min,      -3.0);
    pnh.param<double>("planner/path_generator/delta_s/max",      g_path_cfg.delta_s.max,       3.0);
    pnh.param<double>("planner/path_generator/delta_s/step",     g_path_cfg.delta_s.step,      1.0);
    pnh.param<double>("planner/path_generator/delta_s_dot/min",  g_path_cfg.delta_s_dot.min,  -2.0);
    pnh.param<double>("planner/path_generator/delta_s_dot/max",  g_path_cfg.delta_s_dot.max,   2.0);
    pnh.param<double>("planner/path_generator/delta_s_dot/step", g_path_cfg.delta_s_dot.step,  1.0);
    pnh.param<double>("planner/path_generator/dt",               g_path_cfg.dt,               0.1);

    pnh.param<double>("planner/kinematic_limits/max_lateral_accel",      g_limits.max_lateral_accel,      3.0);
    pnh.param<double>("planner/kinematic_limits/max_longitudinal_accel", g_limits.max_longitudinal_accel, 3.0);
    pnh.param<double>("planner/kinematic_limits/max_curvature",          g_limits.max_curvature,          0.153);

    pnh.param<double>("planner/cost_weights/kj",     g_cost_weights.kj,     1.0);
    pnh.param<double>("planner/cost_weights/kt",     g_cost_weights.kt,     1.0);
    pnh.param<double>("planner/cost_weights/kd",     g_cost_weights.kd,     1.0);
    pnh.param<double>("planner/cost_weights/ks",     g_cost_weights.ks,     1.0);
    pnh.param<double>("planner/cost_weights/ks_dot", g_cost_weights.ks_dot, 1.0);
    pnh.param<double>("planner/cost_weights/klat",   g_cost_weights.klat,   1.0);
    pnh.param<double>("planner/cost_weights/klon",   g_cost_weights.klon,   1.0);

    pnh.param<double>("planner/vehicle_shape/width",  g_vehicle_shape.width,  1.9);
    pnh.param<double>("planner/vehicle_shape/length", g_vehicle_shape.length, 4.5);

    pnh.param<double>("planner/collision_check/safety_margin",      g_collision_cfg.safety_margin,      0.3);
    pnh.param<double>("planner/collision_check/margin_growth_rate", g_collision_cfg.margin_growth_rate, 0.1);

    // FSM이 아직 없을 때 쓰는 고정 목표속도 (config/params.yaml엔 없던 값 -
    // 이 검증 노드 전용 파라미터라 여기서만 기본값을 관리)
    pnh.param<double>("planner/default_target_speed", g_target_speed, 8.0);
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

    LoadParams(pnh);

    std::string waypoint_file;
    pnh.param<std::string>("waypoint_file", waypoint_file, "");
    if (waypoint_file.empty() || !LoadReferenceLine(waypoint_file, g_ref)) {
        ROS_ERROR("[FrenetPlanner] Reference line load failed. Node will keep running "
                   "but will not publish until a valid waypoint_file param is set.");
    } else {
        g_ref_loaded = true;
    }

    // TODO(검증 필요): 아래 두 토픽 이름은 MORAI-ROS 브릿지 launch 설정과
    // 반드시 대조해서 확인할 것 (이 저장소엔 launch 파일이 없어 코드만으로는
    // 확정할 수 없음). /Ego_topic은 mpc_controller가 이미 쓰고 있어 확실함.
    ros::Subscriber ego_sub = nh.subscribe("/Ego_topic", 1, CBEgoState);
    ros::Subscriber obj_sub = nh.subscribe("/Object_topic", 1, CBObjects);

    g_traj_pub = nh.advertise<std_msgs::Float32MultiArray>("/frenet_planner/trajectory", 1);

    double planning_hz = 10.0;  // 논문 Sec.VIII: 100ms 고정 주기
    pnh.param<double>("planner/planning_frequency", planning_hz, planning_hz);
    ros::Timer timer = nh.createTimer(ros::Duration(1.0 / planning_hz), PlanningLoop);

    ROS_INFO("[FrenetPlanner] Subscribed: /Ego_topic /Object_topic");
    ROS_INFO("[FrenetPlanner] Publishing: /frenet_planner/trajectory @ %.1f Hz", planning_hz);
    ROS_INFO("========================================");

    ros::spin();
    return 0;
}
