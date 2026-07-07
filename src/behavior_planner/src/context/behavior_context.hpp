#pragma once
#include <string>
#include <unordered_map>
#include <vector>
#include "behavior_planner/context/behavior_types.hpp"
namespace behavior_planner {
struct HardConstraint { bool force_stop=false, emergency_stop=false, forbid_left_change=false, forbid_right_change=false, forbid_lane_change=false, stop_line_constraint=false; double stop_before_s=1e9, max_speed=1e9; };
struct BehaviorEnable { bool keep=true, follow=false, left_change=false, right_change=false, stop=false, emergency_stop=false; };
struct BehaviorScore { double keep=0, follow=0, left_change=0, right_change=0, stop=0, emergency_stop=0; };
struct BehaviorTarget { std::string current_link_id, route_required_link_id, target_link_id; std::vector<std::string> candidate_link_ids; double desired_speed=0, stop_before_s=1e9; int lead_vehicle_id=-1; double follow_time_gap=1.5, follow_min_gap=5; };
struct BehaviorDebug { std::string selected_reason, hard_rule_reason; std::unordered_map<std::string,double> cost_terms; std::unordered_map<std::string,bool> flags; };
struct BehaviorContextData { double timestamp=0; BehaviorState state=BehaviorState::KEEP_LANE; BehaviorType selected_behavior=BehaviorType::KEEP; HardConstraint hard; BehaviorEnable enable; BehaviorScore score; BehaviorTarget target; BehaviorDebug debug; std::unordered_map<std::string,double> extra_values; std::unordered_map<std::string,bool> extra_flags; };
}
