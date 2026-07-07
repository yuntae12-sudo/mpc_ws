#include "behavior_planner/context/msg_converter.hpp"

#include <ros/ros.h>

namespace behavior_planner {

behavior_planner::BehaviorContext MsgConverter::toMsg(
    const BehaviorContextData& context) {
  behavior_planner::BehaviorContext message;
  message.header.stamp = ros::Time(context.timestamp);
  message.header.frame_id = "map";
  message.selected_behavior = static_cast<uint8_t>(context.selected_behavior);
  message.behavior_state = static_cast<uint8_t>(context.state);
  message.enable_keep = context.enable.keep;
  message.enable_follow = context.enable.follow;
  message.enable_left_change = context.enable.left_change;
  message.enable_right_change = context.enable.right_change;
  message.enable_stop = context.enable.stop;
  message.enable_emergency_stop = context.enable.emergency_stop;
  message.force_stop = context.hard.force_stop;
  message.emergency_stop = context.hard.emergency_stop;
  message.forbid_left_change = context.hard.forbid_left_change;
  message.forbid_right_change = context.hard.forbid_right_change;
  message.forbid_lane_change = context.hard.forbid_lane_change;
  message.desired_speed = context.target.desired_speed;
  message.stop_before_s = context.target.stop_before_s;
  message.max_speed = context.hard.max_speed;
  message.current_link_id = context.target.current_link_id;
  message.route_required_link_id = context.target.route_required_link_id;
  message.target_link_id = context.target.target_link_id;
  message.candidate_link_ids = context.target.candidate_link_ids;
  message.keep_score = context.score.keep;
  message.follow_score = context.score.follow;
  message.left_change_score = context.score.left_change;
  message.right_change_score = context.score.right_change;
  message.stop_score = context.score.stop;
  message.emergency_stop_score = context.score.emergency_stop;
  message.has_lead_vehicle = context.target.lead_vehicle_id >= 0;
  message.lead_vehicle_id = context.target.lead_vehicle_id;
  message.front_gap = context.extra_values.count("front_gap")
                          ? context.extra_values.at("front_gap")
                          : 1e9;
  message.front_relative_speed =
      context.extra_values.count("front_relative_speed")
          ? context.extra_values.at("front_relative_speed")
          : 0.0;
  message.front_ttc = context.extra_values.count("front_ttc")
                          ? context.extra_values.at("front_ttc")
                          : 1e9;
  message.follow_time_gap = context.target.follow_time_gap;
  message.follow_min_gap = context.target.follow_min_gap;
  message.decision_reason = context.debug.selected_reason;
  message.hard_rule_reason = context.debug.hard_rule_reason;
  return message;
}

UrbanFeatureDebug MsgConverter::toDebugMsg(const UrbanFeature& feature) {
  UrbanFeatureDebug message;
  message.header.stamp = ros::Time(feature.timestamp);
  message.header.frame_id = "map";
  message.current_link_id = feature.route.current_link_id;
  message.route_required_link_id = feature.route.route_required_link_id;
  message.ego_s = feature.ego.s;
  message.ego_d = feature.ego.d;
  message.ego_speed = feature.ego.speed;
  message.has_front_vehicle = feature.surrounding.has_front_vehicle;
  message.front_gap = feature.surrounding.front_vehicle.gap;
  message.front_relative_speed =
      feature.surrounding.front_vehicle.relative_speed;
  message.front_ttc = feature.surrounding.front_vehicle.ttc;
  message.left_lane_safe = feature.surrounding.left_lane_safe;
  message.right_lane_safe = feature.surrounding.right_lane_safe;
  message.left_front_gap = feature.surrounding.left_front_gap;
  message.left_rear_gap = feature.surrounding.left_rear_gap;
  message.right_front_gap = feature.surrounding.right_front_gap;
  message.right_rear_gap = feature.surrounding.right_rear_gap;
  message.has_stop_line = feature.signal.has_stop_line;
  message.distance_to_stop_line = feature.signal.distance_to_stop_line;
  message.traffic_light_id = feature.signal.traffic_light_id;
  message.traffic_light_state =
      static_cast<uint8_t>(feature.signal.light_state);
  message.pedestrian_risk = feature.risk.pedestrian_risk;
  message.static_obstacle_ahead = feature.risk.static_obstacle_ahead;
  message.emergency_risk = feature.risk.emergency_risk;
  message.min_ttc = feature.risk.min_ttc;
  message.predicted_front_ttc =
      feature.extra_values.count("predicted_front_ttc")
          ? feature.extra_values.at("predicted_front_ttc")
          : 1e9;
  message.required_deceleration =
      feature.extra_values.count("required_deceleration")
          ? feature.extra_values.at("required_deceleration")
          : 0.0;
  return message;
}

}  // namespace behavior_planner
