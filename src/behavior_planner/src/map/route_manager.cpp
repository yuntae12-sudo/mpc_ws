#include "behavior_planner/map/route_manager.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace behavior_planner {

bool RouteManager::initialize(const MGeoLoader& map,
                              const std::vector<std::string>& ids) {
  map_ = &map;
  ids_ = ids;
  reference_.clear();
  cumulative_.clear();
  segment_link_.clear();
  stops_.clear();
  std::unordered_set<std::string> route_end_nodes;
  double total = 0.0;

  for (const std::string& id : ids) {
    LinkInfo link;
    if (!map.getLink(id, &link)) continue;
    route_end_nodes.insert(link.to_node_id);
    for (const Point2D& point : link.centerline) {
      if (reference_.empty()) {
        reference_.push_back(point);
        cumulative_.push_back(0.0);
        continue;
      }
      const double distance =
          std::hypot(point.x - reference_.back().x,
                     point.y - reference_.back().y);
      if (distance < 1e-6) continue;
      total += distance;
      reference_.push_back(point);
      cumulative_.push_back(total);
      segment_link_.push_back(id);
    }
  }

  // Only accept a stop line whose node is the actual end node of a route link.
  // A distance-only search can select the stop line of an adjacent lane.
  for (StopLineInfo stop : map.stopLines()) {
    if (!route_end_nodes.count(stop.node_id)) continue;
    double route_s = 0.0;
    double route_d = 0.0;
    std::string route_link;
    if (projectEgo(stop.point.x, stop.point.y, &route_s, &route_d,
                   &route_link)) {
      stop.route_s = route_s;
      stop.valid = true;
      stops_.push_back(stop);
    }
  }
  std::sort(stops_.begin(), stops_.end(),
            [](const StopLineInfo& a, const StopLineInfo& b) {
              return a.route_s < b.route_s;
            });
  return reference_.size() > 1;
}

bool RouteManager::projectEgo(double x, double y, double* out_s, double* out_d,
                              std::string* out_id) const {
  if (reference_.size() < 2) return false;
  double best_distance_sq = 1e18;
  double best_s = 0.0;
  double best_d = 0.0;
  size_t best_segment = 0;
  for (size_t i = 1; i < reference_.size(); ++i) {
    const Point2D& a = reference_[i - 1];
    const Point2D& b = reference_[i];
    const double vx = b.x - a.x;
    const double vy = b.y - a.y;
    const double length_sq = vx * vx + vy * vy;
    if (length_sq < 1e-9) continue;
    const double length = std::sqrt(length_sq);
    const double t = std::clamp(
        ((x - a.x) * vx + (y - a.y) * vy) / length_sq, 0.0, 1.0);
    const double dx = x - (a.x + t * vx);
    const double dy = y - (a.y + t * vy);
    const double distance_sq = dx * dx + dy * dy;
    if (distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      best_s = cumulative_[i - 1] + t * length;
      best_d = (vx * dy - vy * dx) / length;
      best_segment = i - 1;
    }
  }
  if (out_s) *out_s = best_s;
  if (out_d) *out_d = best_d;
  if (out_id) {
    *out_id = best_segment < segment_link_.size()
                  ? segment_link_[best_segment]
                  : ids_.back();
  }
  return true;
}

bool RouteManager::findNextStopLine(double ego_s,
                                    StopLineInfo* out_stop_line) const {
  for (const StopLineInfo& stop : stops_) {
    if (stop.route_s >= ego_s) {
      if (out_stop_line) *out_stop_line = stop;
      return true;
    }
  }
  return false;
}

bool RouteManager::getCurrentLinkInfo(const std::string& id,
                                      LinkInfo* out) const {
  return map_ && map_->getLink(id, out);
}

double RouteManager::getSpeedLimit(const std::string& id) const {
  LinkInfo link;
  return getCurrentLinkInfo(id, &link) ? link.max_speed : 0.0;
}

bool RouteManager::getRoutePoint(double s, Point2D* out) const {
  if (!out || reference_.empty() || cumulative_.size() != reference_.size())
    return false;
  if (s <= 0.0) {
    *out = reference_.front();
    return true;
  }
  if (s >= cumulative_.back()) {
    *out = reference_.back();
    return true;
  }
  const auto it = std::lower_bound(cumulative_.begin(), cumulative_.end(), s);
  const size_t i = std::distance(cumulative_.begin(), it);
  if (i == 0) {
    *out = reference_.front();
    return true;
  }
  const double span = cumulative_[i] - cumulative_[i - 1];
  const double t = span > 1e-9 ? (s - cumulative_[i - 1]) / span : 0.0;
  out->x = reference_[i - 1].x +
           t * (reference_[i].x - reference_[i - 1].x);
  out->y = reference_[i - 1].y +
           t * (reference_[i].y - reference_[i - 1].y);
  return true;
}

bool RouteManager::matchMapLink(double x, double y, double yaw,
                                LinkProjection* out,
                                double max_distance) const {
  return map_ && map_->findNearestLink(x, y, yaw, out, max_distance);
}

bool RouteManager::forwardDistance(const std::string& from_link_id,
                                   double from_link_s,
                                   const std::string& to_link_id,
                                   double to_link_s,
                                   double max_distance,
                                   double* out_distance) const {
  return map_ && map_->findForwardDistance(
                     from_link_id, from_link_s, to_link_id, to_link_s,
                     max_distance, out_distance);
}

bool RouteManager::findNextMapStopLine(const std::string& current_link_id,
                                       double current_link_s,
                                       double max_distance,
                                       StopLineInfo* out_stop_line,
                                       double* out_distance) const {
  return map_ && map_->findNextStopLine(current_link_id, current_link_s,
                                       max_distance, out_stop_line,
                                       out_distance);
}

}  // namespace behavior_planner
