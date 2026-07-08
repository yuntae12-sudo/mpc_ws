#include "global/data_logger.hpp"

#include <fstream>
#include <sstream>

bool LoadReferenceLine(const std::string& path, RefLine& out_ref, double max_curvature) {
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

    out_ref = BuildRefLine(wx, wy, max_curvature);
    ROS_INFO("[FrenetPlanner] Loaded %zu waypoints from %s", wx.size(), path.c_str());
    return true;
}

void LoadParams(ros::NodeHandle& pnh,
                 PathGeneratorConfig& path_cfg,
                 KinematicLimits& limits,
                 CostWeights& cost_weights,
                 VehicleShape& vehicle_shape,
                 CollisionCheckConfig& collision_cfg,
                 double& target_speed,
                 BehaviorBridgeConfig& bridge_cfg,
                 double& wheelbase) {
    pnh.param<double>("planner/path_generator/lateral_d1/min",   path_cfg.lateral_d1.min,   -3.0);
    pnh.param<double>("planner/path_generator/lateral_d1/max",   path_cfg.lateral_d1.max,    3.0);
    pnh.param<double>("planner/path_generator/lateral_d1/step",  path_cfg.lateral_d1.step,   0.5);
    pnh.param<double>("planner/path_generator/time_horizon/min", path_cfg.time_horizon.min,  2.0);
    pnh.param<double>("planner/path_generator/time_horizon/max", path_cfg.time_horizon.max,  5.0);
    pnh.param<double>("planner/path_generator/time_horizon/step",path_cfg.time_horizon.step, 1.0);
    pnh.param<double>("planner/path_generator/delta_s/min",      path_cfg.delta_s.min,      -3.0);
    pnh.param<double>("planner/path_generator/delta_s/max",      path_cfg.delta_s.max,       3.0);
    pnh.param<double>("planner/path_generator/delta_s/step",     path_cfg.delta_s.step,      1.0);
    pnh.param<double>("planner/path_generator/delta_s_dot/min",  path_cfg.delta_s_dot.min,  -2.0);
    pnh.param<double>("planner/path_generator/delta_s_dot/max",  path_cfg.delta_s_dot.max,   2.0);
    pnh.param<double>("planner/path_generator/delta_s_dot/step", path_cfg.delta_s_dot.step,  1.0);
    pnh.param<double>("planner/path_generator/dt",               path_cfg.dt,               0.1);

    pnh.param<double>("planner/kinematic_limits/max_lateral_accel",      limits.max_lateral_accel,      3.0);
    pnh.param<double>("planner/kinematic_limits/max_longitudinal_accel", limits.max_longitudinal_accel, 3.0);
    pnh.param<double>("planner/kinematic_limits/max_curvature",          limits.max_curvature,          0.1704);

    pnh.param<double>("planner/cost_weights/kj",     cost_weights.kj,     1.0);
    pnh.param<double>("planner/cost_weights/kt",     cost_weights.kt,     1.0);
    pnh.param<double>("planner/cost_weights/kd",     cost_weights.kd,     1.0);
    pnh.param<double>("planner/cost_weights/ks",     cost_weights.ks,     1.0);
    pnh.param<double>("planner/cost_weights/ks_dot", cost_weights.ks_dot, 1.0);
    pnh.param<double>("planner/cost_weights/klat",   cost_weights.klat,   1.0);
    pnh.param<double>("planner/cost_weights/klon",   cost_weights.klon,   1.0);

    pnh.param<double>("planner/vehicle_shape/width",  vehicle_shape.width,  1.9);
    pnh.param<double>("planner/vehicle_shape/length", vehicle_shape.length, 4.5);

    pnh.param<double>("planner/collision_check/safety_margin",      collision_cfg.safety_margin,      0.3);
    pnh.param<double>("planner/collision_check/margin_growth_rate", collision_cfg.margin_growth_rate, 0.1);
    pnh.param<double>("planner/collision_check/reactive_lookahead", collision_cfg.reactive_lookahead, 8.0);

    // FSM이 아직 없을 때 쓰는 고정 목표속도 (config/params.yaml엔 없던 값 -
    // 이 검증 노드 전용 파라미터라 여기서만 기본값을 관리)
    pnh.param<double>("planner/default_target_speed", target_speed, 8.0);

    // CBEgoState의 kappa 추정(자전거 모델)에 쓰는 축거. mpc_controller/mpc_params.yaml과 동일 값으로 유지.
    pnh.param<double>("planner/wheelbase", wheelbase, 3.0);

    pnh.param<double>("planner/lane_width",              bridge_cfg.lane_width,              3.5);
    pnh.param<double>("planner/behavior_context_timeout",bridge_cfg.context_timeout,         0.5);
    pnh.param<double>("planner/emergency_stop_buffer",   bridge_cfg.emergency_stop_buffer,   12.0);

    pnh.param<double>("planner/cost_weights_emergency/kj",     bridge_cfg.emergency_cost_weights.kj,     1.0);
    pnh.param<double>("planner/cost_weights_emergency/kt",     bridge_cfg.emergency_cost_weights.kt,     20.0);
    pnh.param<double>("planner/cost_weights_emergency/kd",     bridge_cfg.emergency_cost_weights.kd,     1.0);
    pnh.param<double>("planner/cost_weights_emergency/ks",     bridge_cfg.emergency_cost_weights.ks,     1.0);
    pnh.param<double>("planner/cost_weights_emergency/ks_dot", bridge_cfg.emergency_cost_weights.ks_dot, 1.0);
    pnh.param<double>("planner/cost_weights_emergency/klat",   bridge_cfg.emergency_cost_weights.klat,   1.0);
    pnh.param<double>("planner/cost_weights_emergency/klon",   bridge_cfg.emergency_cost_weights.klon,   1.0);
}
