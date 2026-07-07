#include "behavior_planner/visualization/behavior_visualization.hpp"

#include <cmath>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>

namespace behavior_planner {

void BehaviorVisualization::publish(const UrbanFeature& feature,
                                    const BehaviorContextData& context,
                                    const RouteManager& route,
                                    const MGeoLoader& map,
                                    const TrafficLightProvider& traffic_lights) const {
  if (!publisher_) return;
  const ros::Time stamp = ros::Time::now();
  visualization_msgs::MarkerArray array;

  visualization_msgs::Marker route_line;
  route_line.header.frame_id = "map";
  route_line.header.stamp = stamp;
  route_line.ns = "behavior_route";
  route_line.id = 0;
  route_line.type = visualization_msgs::Marker::LINE_STRIP;
  route_line.action = visualization_msgs::Marker::ADD;
  route_line.pose.orientation.w = 1.0;
  route_line.scale.x = 0.35;
  route_line.color.r = 0.1;
  route_line.color.g = 0.8;
  route_line.color.b = 1.0;
  route_line.color.a = 1.0;
  for (const Point2D& point : route.referenceLine()) {
    geometry_msgs::Point ros_point;
    ros_point.x = point.x;
    ros_point.y = point.y;
    ros_point.z = 0.15;
    route_line.points.push_back(ros_point);
  }
  array.markers.push_back(route_line);

  visualization_msgs::Marker ego;
  ego.header = route_line.header;
  ego.ns = "behavior_ego";
  ego.id = 0;
  ego.type = visualization_msgs::Marker::ARROW;
  ego.action = visualization_msgs::Marker::ADD;
  ego.pose.position.x = feature.ego.x;
  ego.pose.position.y = feature.ego.y;
  ego.pose.position.z = 0.4;
  ego.pose.orientation.z = std::sin(feature.ego.yaw * 0.5);
  ego.pose.orientation.w = std::cos(feature.ego.yaw * 0.5);
  ego.scale.x = 3.0;
  ego.scale.y = 1.0;
  ego.scale.z = 1.0;
  ego.color.g = 1.0;
  ego.color.a = 1.0;
  array.markers.push_back(ego);

  visualization_msgs::Marker text;
  text.header = route_line.header;
  text.ns = "behavior_text";
  text.id = 0;
  text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text.action = visualization_msgs::Marker::ADD;
  text.pose.position.x = feature.ego.x;
  text.pose.position.y = feature.ego.y;
  text.pose.position.z = 3.0;
  text.pose.orientation.w = 1.0;
  text.scale.z = 1.2;
  text.color.r = 1.0;
  text.color.g = 1.0;
  text.color.a = 1.0;
  text.text = "behavior=" +
              std::to_string(static_cast<int>(context.selected_behavior)) +
              " current=" + feature.route.current_link_id +
              " route=" + feature.route.route_required_link_id;
  array.markers.push_back(text);

  int signal_marker_id = 0;
  for (const StopLineInfo& map_stop : map.stopLines()) {
    if (!map_stop.valid || map_stop.traffic_light_id.empty()) continue;
    visualization_msgs::Marker signal;
    signal.header = route_line.header;
    signal.ns = "behavior_map_signals";
    signal.id = signal_marker_id++;
    signal.type = visualization_msgs::Marker::SPHERE;
    signal.action = visualization_msgs::Marker::ADD;
    signal.pose.position.x = map_stop.point.x;
    signal.pose.position.y = map_stop.point.y;
    signal.pose.position.z = 0.35;
    signal.pose.orientation.w = 1.0;
    signal.scale.x = 0.8;
    signal.scale.y = 0.8;
    signal.scale.z = 0.8;
    switch (traffic_lights.getState(map_stop.traffic_light_id)) {
      case TrafficLightState::RED:
        signal.color.r = 1.0;
        break;
      case TrafficLightState::YELLOW:
        signal.color.r = 1.0;
        signal.color.g = 0.85;
        break;
      case TrafficLightState::GREEN:
        signal.color.g = 1.0;
        break;
      case TrafficLightState::UNKNOWN:
      default:
        signal.color.r = 0.55;
        signal.color.g = 0.55;
        signal.color.b = 0.55;
        break;
    }
    signal.color.a = 0.9;
    array.markers.push_back(signal);
  }

  if (feature.signal.has_stop_line) {
    visualization_msgs::Marker stop;
    stop.header = route_line.header;
    stop.ns = "behavior_next_stop_line";
    stop.id = 0;
    stop.type = visualization_msgs::Marker::SPHERE;
    stop.action = visualization_msgs::Marker::ADD;
    stop.pose.position.x = feature.signal.stop_line_x;
    stop.pose.position.y = feature.signal.stop_line_y;
    stop.pose.position.z = 0.6;
    stop.pose.orientation.w = 1.0;
    stop.scale.x = 1.4;
    stop.scale.y = 1.4;
    stop.scale.z = 1.4;
    switch (feature.signal.light_state) {
      case TrafficLightState::RED:
        stop.color.r = 1.0;
        break;
      case TrafficLightState::YELLOW:
        stop.color.r = 1.0;
        stop.color.g = 0.85;
        break;
      case TrafficLightState::GREEN:
        stop.color.g = 1.0;
        break;
      case TrafficLightState::UNKNOWN:
      default:
        stop.color.r = 0.55;
        stop.color.g = 0.55;
        stop.color.b = 0.55;
        break;
    }
    stop.color.a = 1.0;
    array.markers.push_back(stop);
  }

  publisher_.publish(array);
}

}  // namespace behavior_planner
