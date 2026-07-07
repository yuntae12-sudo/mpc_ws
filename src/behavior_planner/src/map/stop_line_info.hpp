#pragma once
#include <string>
#include "behavior_planner/map/link_info.hpp"
namespace behavior_planner { struct StopLineInfo { std::string node_id, traffic_light_id; Point2D point; double route_s=1e9; bool valid=false; }; }
