#ifndef PLANNER_GLOBAL_DATA_LOGGER_HPP
#define PLANNER_GLOBAL_DATA_LOGGER_HPP

#include <ros/ros.h>

#include "frenet/ref_line.hpp"
#include "frenet/path_generator.hpp"
#include "frenet/cost.hpp"
#include "frenet/collision_checker.hpp"
#include "global/behavior_bridge.hpp"

// =========================================================
// waypoint 파일("x y" 또는 "x,y") -> RefLine
// =========================================================

bool LoadReferenceLine(const std::string& path, RefLine& out_ref, double max_curvature);

// =========================================================
// params.yaml -> 각 config struct 로드 (config/params.yaml 키 구조와 동일)
// =========================================================

void LoadParams(ros::NodeHandle& pnh,
                 PathGeneratorConfig& path_cfg,
                 KinematicLimits& limits,
                 CostWeights& cost_weights,
                 VehicleShape& vehicle_shape,
                 CollisionCheckConfig& collision_cfg,
                 double& target_speed,
                 BehaviorBridgeConfig& bridge_cfg,
                 double& wheelbase);

#endif
