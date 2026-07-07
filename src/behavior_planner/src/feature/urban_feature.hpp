#pragma once
#include <string>
#include <unordered_map>
#include <vector>
#include "behavior_planner/context/behavior_types.hpp"
namespace behavior_planner {
struct EgoFeature { double x=0,y=0,yaw=0,speed=0,s=0,d=0,map_s=0; bool valid=false; };
struct RouteFeature { std::string current_link_id,target_link_id,route_required_link_id; double speed_limit=0,distance_to_turn=1e9,distance_to_intersection=1e9; bool lane_change_left_allowed=false,lane_change_right_allowed=false,current_link_valid=false; std::string left_target_link_id,right_target_link_id; };
struct SignalFeature { TrafficLightState light_state=TrafficLightState::UNKNOWN; std::string traffic_light_id; bool has_stop_line=false; double stop_line_s=1e9,distance_to_stop_line=1e9,stop_line_x=0,stop_line_y=0; bool stop_line_valid=false; };
struct ObjectFeature { ObjectType type=ObjectType::UNKNOWN; int id=-1; double x=0,y=0,yaw=0,speed=0,accel=0,length=0,width=0,s=0,d=0,link_s=0,gap=1e9,relative_speed=0,ttc=1e9; std::string link_id; bool valid=false; };
struct SurroundingFeature { bool has_front_vehicle=false; ObjectFeature front_vehicle; bool left_lane_safe=false,right_lane_safe=false; double left_front_gap=1e9,left_rear_gap=1e9,right_front_gap=1e9,right_rear_gap=1e9,left_rear_ttc=1e9,right_rear_ttc=1e9; };
struct RiskFeature { bool static_obstacle_ahead=false,pedestrian_risk=false,emergency_risk=false; double min_ttc=1e9,nearest_obstacle_s=1e9,nearest_pedestrian_s=1e9; };
struct UrbanFeature { double timestamp=0; int frame_count=0; EgoFeature ego; RouteFeature route; SignalFeature signal; SurroundingFeature surrounding; RiskFeature risk; std::unordered_map<std::string,double> extra_values; std::unordered_map<std::string,bool> extra_flags; };
struct EgoState { double x=0,y=0,yaw=0,speed=0,stamp=0; };
struct TrackedObject { int id=-1; ObjectType type=ObjectType::UNKNOWN; double x=0,y=0,yaw=0,speed=0,accel=0,length=4.5,width=1.8; };
struct PerceptionObjects { std::vector<TrackedObject> vehicles,pedestrians,obstacles; };
}
