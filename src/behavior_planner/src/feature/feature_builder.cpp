#include "behavior_planner/feature/feature_builder.hpp"

#include <algorithm>
#include <cmath>

namespace behavior_planner {

UrbanFeature FeatureBuilder::build(
    const EgoState& ego, const PerceptionObjects& objects,
    const RouteManager& route_manager,
    const TrafficLightProvider& traffic_light_provider) {
  UrbanFeature feature;
  feature.timestamp = ego.stamp;
  feature.frame_count = ++frame_;
  feature.ego.x = ego.x;
  feature.ego.y = ego.y;
  feature.ego.yaw = ego.yaw;
  feature.ego.speed = ego.speed;

  const bool route_projection_valid = route_manager.projectEgo(
      ego.x, ego.y, &feature.ego.s, &feature.ego.d,
      &feature.route.route_required_link_id);
  LinkProjection map_projection;
  const bool map_match_valid =
      route_manager.matchMapLink(ego.x, ego.y, ego.yaw, &map_projection);
  feature.ego.valid = route_projection_valid && map_match_valid;
  if (map_match_valid) {
    feature.route.current_link_id = map_projection.link_id;
    feature.ego.map_s = map_projection.link_s;
  }

  LinkInfo current_link;
  if (route_manager.getCurrentLinkInfo(feature.route.current_link_id,
                                       &current_link)) {
    feature.route.current_link_valid = true;
    feature.route.speed_limit = current_link.max_speed;
    feature.route.left_target_link_id = current_link.left_dst_link_id;
    feature.route.right_target_link_id = current_link.right_dst_link_id;
    feature.route.lane_change_left_allowed =
        current_link.can_move_left && !current_link.left_dst_link_id.empty();
    feature.route.lane_change_right_allowed =
        current_link.can_move_right && !current_link.right_dst_link_id.empty();
  }

  StopLineInfo stop_line;
  double stop_distance = 1e9;
  if (route_manager.findNextMapStopLine(
          feature.route.current_link_id, feature.ego.map_s, 200.0,
          &stop_line, &stop_distance)) {
    feature.signal.has_stop_line = true;
    feature.signal.stop_line_valid = true;
    feature.signal.stop_line_s = feature.ego.s + stop_distance;
    feature.signal.distance_to_stop_line = stop_distance;
    feature.signal.stop_line_x = stop_line.point.x;
    feature.signal.stop_line_y = stop_line.point.y;
    feature.signal.traffic_light_id = stop_line.traffic_light_id;
    feature.signal.light_state =
        traffic_light_provider.getState(stop_line.traffic_light_id);
  }

  std::vector<ObjectFeature> projected_objects;
  projected_objects.reserve(objects.vehicles.size() + objects.pedestrians.size() +
                            objects.obstacles.size());
  for (const auto& object : objects.vehicles) {
    projected_objects.push_back(
        projector_.project(object, feature.ego, route_manager));
  }
  for (const auto& object : objects.pedestrians) {
    projected_objects.push_back(
        projector_.project(object, feature.ego, route_manager));
  }
  for (const auto& object : objects.obstacles) {
    projected_objects.push_back(
        projector_.project(object, feature.ego, route_manager));
  }

  constexpr double kLaneWidth = 3.5;
  constexpr double kFrontLookahead = 120.0;
  double best_lead_score = 1e9;
  for (ObjectFeature& object : projected_objects) {
    if (!object.valid) continue;
    const double relative_d = object.d - feature.ego.d;
    const bool left_lane =
        (!feature.route.left_target_link_id.empty() &&
         object.link_id == feature.route.left_target_link_id) ||
        std::abs(relative_d - kLaneWidth) < kLaneWidth * 0.5;
    const bool right_lane =
        (!feature.route.right_target_link_id.empty() &&
         object.link_id == feature.route.right_target_link_id) ||
        std::abs(relative_d + kLaneWidth) < kLaneWidth * 0.5;

    // Route-s is appropriate on the global route, but becomes ambiguous when
    // Ego and an object are on an off-route lane near another route segment.
    // Same/adjacent physical links share direction and comparable origins, so
    // use their map-link arc length for longitudinal front/rear classification.
    const bool physical_neighbor_lane =
        object.link_id == feature.route.current_link_id ||
        object.link_id == feature.route.left_target_link_id ||
        object.link_id == feature.route.right_target_link_id;
    if (physical_neighbor_lane) {
      object.gap = object.link_s - feature.ego.map_s;
      object.ttc = object.gap > 0.0 && object.relative_speed > 0.0
                       ? object.gap / object.relative_speed
                       : 1e9;
    }

    const double dx = object.x - feature.ego.x;
    const double dy = object.y - feature.ego.y;
    const double cos_yaw = std::cos(feature.ego.yaw);
    const double sin_yaw = std::sin(feature.ego.yaw);
    const double ego_longitudinal = cos_yaw * dx + sin_yaw * dy;
    const double ego_lateral = -sin_yaw * dx + cos_yaw * dy;
    const double euclidean_distance = std::hypot(dx, dy);
    const bool same_direction =
        std::cos(object.yaw - feature.ego.yaw) > 0.5;
    double topology_gap = 1e9;
    const bool forward_in_topology = route_manager.forwardDistance(
        feature.route.current_link_id, feature.ego.map_s, object.link_id,
        object.link_s, kFrontLookahead, &topology_gap);
    const double corridor_half_width =
        std::min(12.0, 2.5 + 0.12 * topology_gap);
    const bool geometry_front_gate =
        ego_longitudinal > -2.0 && euclidean_distance <= kFrontLookahead &&
        std::abs(ego_lateral) <= corridor_half_width && same_direction;
    const bool current_lane_front = forward_in_topology && geometry_front_gate;

    if (current_lane_front) {
      object.gap = topology_gap;
      object.ttc = object.gap > 0.0 && object.relative_speed > 0.0
                       ? object.gap / object.relative_speed
                       : 1e9;
    }

    if (object.type == ObjectType::VEHICLE && current_lane_front &&
        object.gap > 0.0) {
      // A small continuity bonus prevents lead selection from flickering when
      // two vehicles have nearly equal gaps around a link boundary.
      const double lead_score =
          object.gap - (object.id == last_lead_vehicle_id_ ? 4.0 : 0.0);
      if (lead_score < best_lead_score) {
        best_lead_score = lead_score;
        feature.surrounding.has_front_vehicle = true;
        feature.surrounding.front_vehicle = object;
      }
    }
    if (object.type == ObjectType::PEDESTRIAN && object.gap > 0.0 &&
        object.gap < 15.0 && std::abs(relative_d) < kLaneWidth) {
      feature.risk.pedestrian_risk = true;
      feature.risk.nearest_pedestrian_s =
          std::min(feature.risk.nearest_pedestrian_s, object.s);
    }
    if (object.type == ObjectType::STATIC_OBSTACLE && current_lane_front &&
        object.gap > 0.0 && object.gap < 30.0) {
      feature.risk.static_obstacle_ahead = true;
      feature.risk.nearest_obstacle_s =
          std::min(feature.risk.nearest_obstacle_s, object.s);
    }
    if (current_lane_front && object.gap > 0.0) {
      feature.risk.min_ttc = std::min(feature.risk.min_ttc, object.ttc);
    }

    if (left_lane) {
      if (object.gap >= 0.0) {
        feature.surrounding.left_front_gap =
            std::min(feature.surrounding.left_front_gap, object.gap);
      } else {
        feature.surrounding.left_rear_gap =
            std::min(feature.surrounding.left_rear_gap, -object.gap);
      }
    }
    if (right_lane) {
      if (object.gap >= 0.0) {
        feature.surrounding.right_front_gap =
            std::min(feature.surrounding.right_front_gap, object.gap);
      } else {
        feature.surrounding.right_rear_gap =
            std::min(feature.surrounding.right_rear_gap, -object.gap);
      }
    }
  }

  last_lead_vehicle_id_ = feature.surrounding.has_front_vehicle
                              ? feature.surrounding.front_vehicle.id
                              : -1;

  feature.surrounding.left_lane_safe =
      feature.route.lane_change_left_allowed &&
      feature.surrounding.left_front_gap >= 12.0 &&
      feature.surrounding.left_rear_gap >= 15.0;
  feature.surrounding.right_lane_safe =
      feature.route.lane_change_right_allowed &&
      feature.surrounding.right_front_gap >= 12.0 &&
      feature.surrounding.right_rear_gap >= 15.0;
  double predicted_front_ttc = 1e9;
  double required_deceleration = 0.0;
  if (feature.surrounding.has_front_vehicle) {
    const ObjectFeature& lead = feature.surrounding.front_vehicle;
    const double closing_speed =
        std::max(0.0, feature.ego.speed - lead.speed);
    const double available_braking_distance =
        std::max(0.5, lead.gap - 8.0);
    required_deceleration =
        closing_speed * closing_speed / (2.0 * available_braking_distance);

    constexpr double kPredictionHorizon = 1.0;
    const double predicted_lead_speed =
        std::max(0.0, lead.speed +
                          std::min(0.0, lead.accel) * kPredictionHorizon);
    const double predicted_closing_speed =
        feature.ego.speed - predicted_lead_speed;
    if (predicted_closing_speed > 0.0) {
      predicted_front_ttc = lead.gap / predicted_closing_speed;
      feature.risk.min_ttc =
          std::min(feature.risk.min_ttc, predicted_front_ttc);
    }
  }
  feature.extra_values["predicted_front_ttc"] = predicted_front_ttc;
  feature.extra_values["required_deceleration"] = required_deceleration;
  feature.risk.emergency_risk =
      feature.risk.min_ttc < 1.0 || predicted_front_ttc < 1.2 ||
      required_deceleration > 6.0;
  return feature;
}

}  // namespace behavior_planner
