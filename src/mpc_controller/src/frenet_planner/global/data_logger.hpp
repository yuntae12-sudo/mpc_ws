#pragma once

#include <string>

#include "frenet_planner/frenet/ref_line.hpp"
#include "frenet_planner/frenet/path_generator.hpp"
#include "frenet_planner/frenet/cost.hpp"
#include "frenet_planner/frenet/collision_checker.hpp"

// =========================================================
// 곡률 기반 사전 감속 (mpc_controller의 옛 planner/path_planner.cpp 로직을
// Frenet Frame Planner 쪽으로 이식한 것). lookahead 구간 내 최대 곡률로
// velocity-keeping 목표속도를 정한다.
// =========================================================
struct CurveSpeedConfig {
    double target_vel        = 10.0;
    double curve_vel_sharp   = 4.0;
    double curve_vel_mid     = 5.5;
    double curve_vel_mild    = 7.0;
    double curve_th_sharp    = 0.080;
    double curve_th_mid      = 0.035;
    double curve_th_mild     = 0.015;
    double curve_lookahead_m = 15.0;
};

// waypoint 파일("x y" 또는 "x,y") -> RefLine
bool LoadReferenceLine(const std::string& path, RefLine& out_ref, double max_curvature);

// params.yaml -> 각 config struct 로드 (yaml-cpp 기반, ROS 의존성 없음)
void LoadParams(const std::string& yaml_path,
                PathGeneratorConfig& path_cfg,
                KinematicLimits& limits,
                CostWeights& cost_weights,
                VehicleShape& vehicle_shape,
                CollisionCheckConfig& collision_cfg,
                CurveSpeedConfig& curve_speed_cfg,
                double& wheelbase,
                double& lane_width,
                std::string& waypoint_file);
