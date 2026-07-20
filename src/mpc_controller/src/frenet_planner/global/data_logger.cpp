#include "frenet_planner/global/data_logger.hpp"

#include <cstdio>
#include <fstream>
#include <sstream>

#include <yaml-cpp/yaml.h>

bool LoadReferenceLine(const std::string& path, RefLine& out_ref, double max_curvature) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::printf("[FrenetPlanner] Failed to open waypoint file: %s\n", path.c_str());
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
        std::printf("[FrenetPlanner] Waypoint file has fewer than 2 valid points: %s\n", path.c_str());
        return false;
    }

    out_ref = BuildRefLine(wx, wy, max_curvature);
    std::printf("[FrenetPlanner] Loaded %zu waypoints from %s\n", wx.size(), path.c_str());
    return true;
}

namespace {

template <typename T>
bool loadInto(const YAML::Node& root, const std::string& dotted_key, T& out) {
    YAML::Node node = YAML::Clone(root);
    size_t start = 0;
    while (true) {
        size_t slash = dotted_key.find('/', start);
        std::string key = dotted_key.substr(start, slash == std::string::npos ? std::string::npos : slash - start);
        if (!node[key]) return false;
        node = node[key];
        if (slash == std::string::npos) break;
        start = slash + 1;
    }
    out = node.as<T>();
    return true;
}

}  // namespace

void LoadParams(const std::string& yaml_path,
                 PathGeneratorConfig& path_cfg,
                 KinematicLimits& limits,
                 CostWeights& cost_weights,
                 VehicleShape& vehicle_shape,
                 CollisionCheckConfig& collision_cfg,
                 CurveSpeedConfig& curve_speed_cfg,
                 double& wheelbase,
                 double& lane_width,
                 std::string& waypoint_file) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const std::exception& e) {
        std::printf("[FrenetPlanner] Failed to load yaml '%s': %s (기본값으로 진행)\n",
                    yaml_path.c_str(), e.what());
        return;
    }

    loadInto(root, "planner/path_generator/lateral_d1/min",   path_cfg.lateral_d1.min);
    loadInto(root, "planner/path_generator/lateral_d1/max",   path_cfg.lateral_d1.max);
    loadInto(root, "planner/path_generator/lateral_d1/step",  path_cfg.lateral_d1.step);
    loadInto(root, "planner/path_generator/time_horizon/min", path_cfg.time_horizon.min);
    loadInto(root, "planner/path_generator/time_horizon/max", path_cfg.time_horizon.max);
    loadInto(root, "planner/path_generator/time_horizon/step",path_cfg.time_horizon.step);
    loadInto(root, "planner/path_generator/delta_s/min",      path_cfg.delta_s.min);
    loadInto(root, "planner/path_generator/delta_s/max",      path_cfg.delta_s.max);
    loadInto(root, "planner/path_generator/delta_s/step",     path_cfg.delta_s.step);
    loadInto(root, "planner/path_generator/delta_s_dot/min",  path_cfg.delta_s_dot.min);
    loadInto(root, "planner/path_generator/delta_s_dot/max",  path_cfg.delta_s_dot.max);
    loadInto(root, "planner/path_generator/delta_s_dot/step", path_cfg.delta_s_dot.step);
    loadInto(root, "planner/path_generator/dt",               path_cfg.dt);

    loadInto(root, "planner/kinematic_limits/max_lateral_accel",      limits.max_lateral_accel);
    loadInto(root, "planner/kinematic_limits/max_longitudinal_accel", limits.max_longitudinal_accel);
    loadInto(root, "planner/kinematic_limits/max_curvature",          limits.max_curvature);

    loadInto(root, "planner/cost_weights/kj",     cost_weights.kj);
    loadInto(root, "planner/cost_weights/kt",     cost_weights.kt);
    loadInto(root, "planner/cost_weights/kd",     cost_weights.kd);
    loadInto(root, "planner/cost_weights/ks",     cost_weights.ks);
    loadInto(root, "planner/cost_weights/ks_dot", cost_weights.ks_dot);
    loadInto(root, "planner/cost_weights/klat",   cost_weights.klat);
    loadInto(root, "planner/cost_weights/klon",   cost_weights.klon);

    loadInto(root, "planner/vehicle_shape/width",  vehicle_shape.width);
    loadInto(root, "planner/vehicle_shape/length", vehicle_shape.length);

    loadInto(root, "planner/collision_check/safety_margin",      collision_cfg.safety_margin);
    loadInto(root, "planner/collision_check/margin_growth_rate", collision_cfg.margin_growth_rate);
    loadInto(root, "planner/collision_check/reactive_lookahead", collision_cfg.reactive_lookahead);

    loadInto(root, "planner/curve_speed/target_vel",       curve_speed_cfg.target_vel);
    loadInto(root, "planner/curve_speed/curve_vel_sharp",  curve_speed_cfg.curve_vel_sharp);
    loadInto(root, "planner/curve_speed/curve_vel_mid",    curve_speed_cfg.curve_vel_mid);
    loadInto(root, "planner/curve_speed/curve_vel_mild",   curve_speed_cfg.curve_vel_mild);
    loadInto(root, "planner/curve_speed/curve_th_sharp",   curve_speed_cfg.curve_th_sharp);
    loadInto(root, "planner/curve_speed/curve_th_mid",     curve_speed_cfg.curve_th_mid);
    loadInto(root, "planner/curve_speed/curve_th_mild",    curve_speed_cfg.curve_th_mild);
    loadInto(root, "planner/curve_speed/curve_lookahead_m",curve_speed_cfg.curve_lookahead_m);

    loadInto(root, "planner/wheelbase",   wheelbase);
    loadInto(root, "planner/lane_width",  lane_width);
    loadInto(root, "waypoint_file",       waypoint_file);
}
