#include "behavior_planner/map/mgeo_loader.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

namespace behavior_planner {
namespace {

std::string textValue(const YAML::Node& node, const char* key,
                      const std::string& fallback = "") {
  const YAML::Node value = node[key];
  if (!value || !value.IsScalar()) return fallback;
  try {
    return value.as<std::string>();
  } catch (const YAML::Exception&) {
    return fallback;
  }
}

double numberValue(const YAML::Node& node, const char* key, double fallback) {
  const YAML::Node value = node[key];
  if (!value || !value.IsScalar()) return fallback;
  try {
    return value.as<double>();
  } catch (const YAML::Exception&) {
    return fallback;
  }
}

bool boolValue(const YAML::Node& node, const char* key, bool fallback) {
  const YAML::Node value = node[key];
  if (!value || !value.IsScalar()) return fallback;
  try {
    return value.as<bool>();
  } catch (const YAML::Exception&) {
    return fallback;
  }
}

int intValue(const YAML::Node& node, const char* key, int fallback) {
  const YAML::Node value = node[key];
  if (!value || !value.IsScalar()) return fallback;
  try {
    return value.as<int>();
  } catch (const YAML::Exception&) {
    return fallback;
  }
}

}  // namespace

bool MGeoLoader::load(const std::string& link_path,
                      const std::string& node_path) {
  links_.clear();
  outgoing_links_.clear();
  stop_lines_by_node_.clear();
  stop_lines_.clear();

  try {
    // JSON is a YAML 1.2 subset, so yaml-cpp can parse the MORAI JSON files.
    // This avoids requiring an optional nlohmann/json installation.
    const YAML::Node link_root = YAML::LoadFile(link_path);
    const YAML::Node node_root = YAML::LoadFile(node_path);
    const YAML::Node link_array = link_root["links"] ? link_root["links"]
                                                      : link_root;
    const YAML::Node node_array = node_root["nodes"] ? node_root["nodes"]
                                                      : node_root;
    if (!link_array.IsSequence() || !node_array.IsSequence()) {
      ROS_ERROR("MGeo link_set/node_set root must be an array");
      return false;
    }

    for (const YAML::Node& value : link_array) {
      LinkInfo link;
      link.id = textValue(value, "idx", textValue(value, "id"));
      link.from_node_id = textValue(value, "from_node_idx");
      link.to_node_id = textValue(value, "to_node_idx");
      // MORAI MGeo speed limits are km/h; internal planner units are m/s.
      link.max_speed = numberValue(value, "max_speed", 0.0) / 3.6;
      link.min_speed = numberValue(value, "min_speed", 0.0) / 3.6;
      link.width_start = numberValue(value, "width_start", 3.5);
      link.width_end = numberValue(value, "width_end", 3.5);
      link.can_move_left = boolValue(value, "can_move_left_lane", false);
      link.can_move_right = boolValue(value, "can_move_right_lane", false);
      link.left_dst_link_id =
          textValue(value, "left_lane_change_dst_link_idx");
      link.right_dst_link_id =
          textValue(value, "right_lane_change_dst_link_idx");
      link.related_signal = textValue(value, "related_signal");
      link.road_id = textValue(value, "road_id");
      link.ego_lane = intValue(value, "ego_lane", -1);

      const YAML::Node points = value["points"];
      if (points && points.IsSequence()) {
        for (const YAML::Node& point : points) {
          if (!point.IsSequence() || point.size() < 2) continue;
          try {
            link.centerline.push_back(
                {point[0].as<double>(), point[1].as<double>()});
          } catch (const YAML::Exception&) {
            ROS_WARN_STREAM("Invalid point in link " << link.id);
          }
        }
      }
      for (size_t i = 1; i < link.centerline.size(); ++i) {
        link.link_length +=
            std::hypot(link.centerline[i].x - link.centerline[i - 1].x,
                       link.centerline[i].y - link.centerline[i - 1].y);
      }
      if (!link.id.empty()) links_[link.id] = std::move(link);
    }

    for (const auto& entry : links_) {
      outgoing_links_[entry.second.from_node_id].push_back(entry.first);
    }

    for (const YAML::Node& value : node_array) {
      if (!boolValue(value, "on_stop_line", false)) continue;
      StopLineInfo stop_line;
      stop_line.node_id = textValue(value, "idx", textValue(value, "id"));
      stop_line.traffic_light_id = textValue(value, "traffic_light_id");
      const YAML::Node point = value["point"];
      if (point && point.IsSequence() && point.size() >= 2) {
        stop_line.point = {point[0].as<double>(), point[1].as<double>()};
        stop_line.valid = true;
      }
      stop_lines_.push_back(stop_line);
      stop_lines_by_node_[stop_line.node_id].push_back(stop_line);
    }

    ROS_INFO_STREAM("Loaded " << links_.size() << " links and "
                               << stop_lines_.size() << " stop lines");
    return !links_.empty();
  } catch (const YAML::Exception& error) {
    ROS_ERROR_STREAM("MGeo parse failed: " << error.what());
  } catch (const std::exception& error) {
    ROS_ERROR_STREAM("MGeo loading failed: " << error.what());
  }

  links_.clear();
  stop_lines_.clear();
  return false;
}

bool MGeoLoader::getLink(const std::string& id, LinkInfo* out) const {
  const auto it = links_.find(id);
  if (it == links_.end() || !out) return false;
  *out = it->second;
  return true;
}

bool MGeoLoader::findNearestLink(double x, double y, double yaw,
                                 LinkProjection* out,
                                 double max_distance) const {
  if (!out) return false;
  double best_score = 1e18;
  LinkProjection best;

  for (const auto& entry : links_) {
    const LinkInfo& link = entry.second;
    double cumulative_s = 0.0;
    for (size_t i = 1; i < link.centerline.size(); ++i) {
      const Point2D& a = link.centerline[i - 1];
      const Point2D& b = link.centerline[i];
      const double vx = b.x - a.x;
      const double vy = b.y - a.y;
      const double length_sq = vx * vx + vy * vy;
      if (length_sq < 1e-9) continue;
      const double length = std::sqrt(length_sq);
      const double raw_t = ((x - a.x) * vx + (y - a.y) * vy) / length_sq;
      const double t = std::max(0.0, std::min(1.0, raw_t));
      const double px = a.x + t * vx;
      const double py = a.y + t * vy;
      const double dx = x - px;
      const double dy = y - py;
      const double distance_sq = dx * dx + dy * dy;
      const double segment_yaw = std::atan2(vy, vx);
      const double heading_penalty = 2.0 * (1.0 - std::cos(yaw - segment_yaw));
      const double score = distance_sq + heading_penalty;
      if (score < best_score) {
        best_score = score;
        best.link_id = link.id;
        best.link_s = cumulative_s + t * length;
        best.d = (vx * dy - vy * dx) / length;
        best.distance = std::sqrt(distance_sq);
      }
      cumulative_s += length;
    }
  }

  if (best.link_id.empty() || best.distance > max_distance) return false;
  *out = best;
  return true;
}

bool MGeoLoader::findForwardDistance(const std::string& from_link_id,
                                     double from_link_s,
                                     const std::string& to_link_id,
                                     double to_link_s,
                                     double max_distance,
                                     double* out_distance) const {
  if (!out_distance) return false;
  const auto from_it = links_.find(from_link_id);
  const auto to_it = links_.find(to_link_id);
  if (from_it == links_.end() || to_it == links_.end()) return false;

  if (from_link_id == to_link_id) {
    const double distance = to_link_s - from_link_s;
    if (distance < 0.0 || distance > max_distance) return false;
    *out_distance = distance;
    return true;
  }

  using QueueEntry = std::pair<double, std::string>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>,
                      std::greater<QueueEntry>> queue;
  std::unordered_map<std::string, double> best_cost;
  const double remaining =
      std::max(0.0, from_it->second.link_length - from_link_s);
  if (remaining > max_distance) return false;

  const auto first_links = outgoing_links_.find(from_it->second.to_node_id);
  if (first_links == outgoing_links_.end()) return false;
  for (const std::string& next_id : first_links->second) {
    if (next_id == to_link_id) {
      const double distance = remaining + to_link_s;
      if (distance <= max_distance) {
        *out_distance = distance;
        return true;
      }
    }
    queue.push({remaining, next_id});
  }

  while (!queue.empty()) {
    const auto [cost_to_start, link_id] = queue.top();
    queue.pop();
    if (cost_to_start > max_distance) continue;
    const auto old = best_cost.find(link_id);
    if (old != best_cost.end() && old->second <= cost_to_start) continue;
    best_cost[link_id] = cost_to_start;
    const LinkInfo& link = links_.at(link_id);
    const double cost_to_end = cost_to_start + link.link_length;
    if (cost_to_end > max_distance) continue;
    const auto next_links = outgoing_links_.find(link.to_node_id);
    if (next_links == outgoing_links_.end()) continue;
    for (const std::string& next_id : next_links->second) {
      if (next_id == to_link_id) {
        const double distance = cost_to_end + to_link_s;
        if (distance <= max_distance) {
          *out_distance = distance;
          return true;
        }
      }
      queue.push({cost_to_end, next_id});
    }
  }
  return false;
}

bool MGeoLoader::findNextStopLine(const std::string& current_link_id,
                                  double current_link_s,
                                  double max_distance,
                                  StopLineInfo* out_stop_line,
                                  double* out_distance) const {
  if (!out_stop_line || !out_distance) return false;
  const auto current_it = links_.find(current_link_id);
  if (current_it == links_.end()) return false;

  using QueueEntry = std::pair<double, std::string>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>,
                      std::greater<QueueEntry>> queue;
  std::unordered_map<std::string, double> best_cost;
  const double remaining =
      std::max(0.0, current_it->second.link_length - current_link_s);

  const auto current_stop =
      stop_lines_by_node_.find(current_it->second.to_node_id);
  if (current_stop != stop_lines_by_node_.end() &&
      remaining <= max_distance && !current_stop->second.empty()) {
    *out_stop_line = current_stop->second.front();
    *out_distance = remaining;
    return true;
  }

  const auto first_links = outgoing_links_.find(current_it->second.to_node_id);
  if (first_links == outgoing_links_.end()) return false;
  for (const std::string& next_id : first_links->second) {
    queue.push({remaining, next_id});
  }

  bool found = false;
  double best_stop_distance = 1e9;
  StopLineInfo best_stop;
  while (!queue.empty()) {
    const auto [cost_to_start, link_id] = queue.top();
    queue.pop();
    if (cost_to_start > max_distance || cost_to_start >= best_stop_distance)
      continue;
    const auto old = best_cost.find(link_id);
    if (old != best_cost.end() && old->second <= cost_to_start) continue;
    best_cost[link_id] = cost_to_start;
    const LinkInfo& link = links_.at(link_id);
    const double cost_to_end = cost_to_start + link.link_length;
    if (cost_to_end > max_distance) continue;

    const auto stop = stop_lines_by_node_.find(link.to_node_id);
    if (stop != stop_lines_by_node_.end() && !stop->second.empty()) {
      found = true;
      best_stop_distance = cost_to_end;
      best_stop = stop->second.front();
      continue;
    }
    const auto next_links = outgoing_links_.find(link.to_node_id);
    if (next_links == outgoing_links_.end()) continue;
    for (const std::string& next_id : next_links->second) {
      queue.push({cost_to_end, next_id});
    }
  }
  if (!found) return false;
  *out_stop_line = best_stop;
  *out_distance = best_stop_distance;
  return true;
}

}  // namespace behavior_planner
