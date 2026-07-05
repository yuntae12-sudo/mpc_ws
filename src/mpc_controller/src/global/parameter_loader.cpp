#include "parameter_loader.hpp"
#include "global.hpp"
#include "utils.hpp"


// ========================================
// ROS 파라미터 → mpc_params
//   yaml 구조와 키 이름을 일치시킴
//   (launch 에서 rosparam load 로 namespace 에 올린다는 전제)
// ========================================
void loadMPCParameters(ros::NodeHandle& pnh)
{
    auto& p = mpc_params;

    // 예측
    pnh.param<int>   ("mpc/horizon",            p.horizon,           p.horizon);
    pnh.param<double>("mpc/dt",                 p.dt,                p.dt);
    pnh.param<int>   ("mpc/max_iterations",     p.max_iterations,    p.max_iterations);
    pnh.param<double>("mpc/wheelbase",          p.wheelbase,         p.wheelbase);
    pnh.param<double>("control_frequency",      p.control_frequency, p.control_frequency);

    // 제약
    pnh.param<double>("constraints/steering/max_angle", p.steering_max,      p.steering_max);
    pnh.param<double>("constraints/steering/max_rate",  p.steering_rate_max, p.steering_rate_max);
    pnh.param<double>("constraints/acceleration/max_forward", p.accel_max, p.accel_max);
    pnh.param<double>("constraints/acceleration/max_reverse", p.accel_min, p.accel_min);
    pnh.param<double>("constraints/velocity/min",       p.vel_min, p.vel_min);
    pnh.param<double>("constraints/velocity/max",       p.vel_max, p.vel_max);

    // 비용 가중치
    pnh.param<double>("cost/path_error",     p.weight_path_error,    p.weight_path_error);
    pnh.param<double>("cost/heading_error",  p.weight_heading_error, p.weight_heading_error);
    pnh.param<double>("cost/speed_error",    p.weight_speed_error,   p.weight_speed_error);
    pnh.param<double>("cost/control",        p.weight_control,       p.weight_control);
    pnh.param<double>("cost/control_rate",   p.weight_control_rate,  p.weight_control_rate);
    pnh.param<double>("cost/terminal",       p.weight_terminal,      p.weight_terminal);

    // 솔버
    pnh.param<double>("solver/lr_init",          p.lr_init,          p.lr_init);
    pnh.param<double>("solver/lr_min",           p.lr_min,           p.lr_min);
    pnh.param<double>("solver/convergence_eps",  p.convergence_eps,  p.convergence_eps);
    pnh.param<int>   ("solver/line_search_steps",p.line_search_steps,p.line_search_steps);

    // 속도 계획
    pnh.param<int>   ("planning/ref_window",      p.ref_window,      p.ref_window);
    pnh.param<double>("planning/target_vel",      p.target_vel,      p.target_vel);
    pnh.param<double>("planning/curve_vel_sharp", p.curve_vel_sharp, p.curve_vel_sharp);
    pnh.param<double>("planning/curve_vel_mid",   p.curve_vel_mid,   p.curve_vel_mid);
    pnh.param<double>("planning/curve_vel_mild",  p.curve_vel_mild,  p.curve_vel_mild);
    pnh.param<double>("planning/curve_th_sharp",  p.curve_th_sharp,  p.curve_th_sharp);
    pnh.param<double>("planning/curve_th_mid",    p.curve_th_mid,    p.curve_th_mid);
    pnh.param<double>("planning/curve_th_mild",   p.curve_th_mild,   p.curve_th_mild);

    // CSV 파일 경로 (절대/상대 모두 허용)
    pnh.param<std::string>("waypoint_file", g_waypoint_file_path, g_waypoint_file_path);
    pnh.param<std::string>("ref_file",      g_ref_file_path,      g_ref_file_path);

    ROS_INFO("[MPC] Params: horizon=%d dt=%.2f max_iter=%d wheelbase=%.2f freq=%.1fHz",
             p.horizon, p.dt, p.max_iterations, p.wheelbase, p.control_frequency);
    ROS_INFO("[MPC] Weights: path=%.2f head=%.2f speed=%.2f term=%.2f",
             p.weight_path_error, p.weight_heading_error, p.weight_speed_error,
             p.weight_terminal);
    ROS_INFO("[MPC] Waypoint file: %s", g_waypoint_file_path.c_str());
}

// ========================================
// 3점 원 근사로 곡률 계산
//   k = 2 * |cross| / (a*b*c)  where a,b,c 는 삼각형 세 변
// ========================================
void computeWaypointCurvatures()
{
    if (waypoints.size() < 3) return;

    auto curvAt = [&](size_t i)->double {
        const size_t gap = 25;
        if (i < gap || i + gap >= waypoints.size()) return 0.0;
        const auto& p0 = waypoints[i-gap];
        const auto& p1 = waypoints[i];
        const auto& p2 = waypoints[i+gap];
        double a = std::hypot(p1.x-p0.x, p1.y-p0.y);
        double b = std::hypot(p2.x-p1.x, p2.y-p1.y);
        double c = std::hypot(p2.x-p0.x, p2.y-p0.y);
        if (a < 1e-6 || b < 1e-6 || c < 1e-6) return 0.0;
        double cross = std::fabs((p1.x-p0.x)*(p2.y-p0.y) - (p1.y-p0.y)*(p2.x-p0.x));
        return 2.0 * cross / (a*b*c);
    };

    for (size_t i = 0; i < waypoints.size(); ++i) {
        waypoints[i].curvature = curvAt(i);
    }
    waypoints.front().curvature = waypoints.size() > 1 ? waypoints[1].curvature : 0.0;
    waypoints.back().curvature  = waypoints.size() > 1 ? waypoints[waypoints.size()-2].curvature : 0.0;
}

// ========================================
// CSV 에서 ref + waypoints 로드
//   PlanningControl 의 loadWaypoints 와 호환:
//     ref.txt:     "lat0 lon0 h0"
//     waypoint csv: 각 라인 "x y ..."  또는 "x,y,..."
// ========================================
bool loadWaypoints()
{
    waypoints.clear();

    // 1) ref point
    ROS_INFO("[MPC] Opening ref file: %s", g_ref_file_path.c_str());
    std::ifstream ref_file(g_ref_file_path);
    if (!ref_file.is_open()) {
        ROS_WARN("[MPC] Failed to open ref file: %s  (GPS 콜백 첫 수신 시 ref 설정)",
                 g_ref_file_path.c_str());
        // ref 파일이 없어도 GPS 첫 수신에서 자동 초기화되므로 계속 진행
    } else {
        ref_file >> coord_ref.lat0 >> coord_ref.lon0 >> coord_ref.h0;
        ref_file.close();

        wgs84ToECEF(coord_ref.lat0, coord_ref.lon0, coord_ref.h0,
                    coord_ref.x0_ecef, coord_ref.y0_ecef, coord_ref.z0_ecef);
        coord_ref_initialized = true;
        ROS_INFO("[MPC] Ref point: lat=%.8f lon=%.8f h=%.3f",
                 coord_ref.lat0, coord_ref.lon0, coord_ref.h0);
    }

    // 2) waypoint csv
    ROS_INFO("[MPC] Opening waypoint file: %s", g_waypoint_file_path.c_str());
    std::ifstream path_file(g_waypoint_file_path);
    if (!path_file.is_open()) {
        ROS_ERROR("[MPC] Failed to open waypoint file: %s", g_waypoint_file_path.c_str());
        return false;
    }

    std::string line;
    while (std::getline(path_file, line)) {
        if (line.empty()) continue;

        // 공백 우선
        {
            std::stringstream ss(line);
            double x, y;
            if (ss >> x >> y) {
                Waypoint wp;
                wp.x = x; wp.y = y; wp.curvature = 0.0;
                waypoints.push_back(wp);
                continue;
            }
        }

        // CSV (콤마 구분)
        std::stringstream ss(line);
        std::string val;
        std::vector<double> row;
        while (std::getline(ss, val, ',')) {
            // trim
            val.erase(0, val.find_first_not_of(" \t\r\n"));
            if (!val.empty())
                val.erase(val.find_last_not_of(" \t\r\n") + 1);
            if (val.empty()) continue;
            try { row.push_back(std::stod(val)); } catch (...) { /* header skip */ }
        }
        if (row.size() >= 2) {
            Waypoint wp;
            wp.x = row[0]; wp.y = row[1]; wp.curvature = 0.0;
            waypoints.push_back(wp);
        }
    }
    path_file.close();

    if (waypoints.size() < 3) {
        ROS_ERROR("[MPC] Not enough waypoints! (size=%zu)", waypoints.size());
        return false;
    }

    computeWaypointCurvatures();

    ROS_INFO("[MPC] Loaded %zu waypoints", waypoints.size());
    return true;
}
