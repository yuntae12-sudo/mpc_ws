#pragma once

#include <string>
#include <vector>

#include "behavior_planner/map/mgeo_loader.hpp"

namespace behavior_planner {

class RouteManager {
 public:
  bool initialize(const MGeoLoader& map,
                  const std::vector<std::string>& route_link_ids);
  bool projectEgo(double x, double y, double* out_s, double* out_d,
                  std::string* out_link_id) const;
  bool findNextStopLine(double ego_s, StopLineInfo* out_stop_line) const;
  bool getCurrentLinkInfo(const std::string& current_link_id,
                          LinkInfo* out) const;
  double getSpeedLimit(const std::string& current_link_id) const;
  bool getRoutePoint(double route_s, Point2D* out) const;
  bool matchMapLink(double x, double y, double yaw,
                    LinkProjection* out,
                    double max_distance = 12.0) const;
  bool forwardDistance(const std::string& from_link_id,
                       double from_link_s,
                       const std::string& to_link_id,
                       double to_link_s,
                       double max_distance,
                       double* out_distance) const;
  bool findNextMapStopLine(const std::string& current_link_id,
                           double current_link_s,
                           double max_distance,
                           StopLineInfo* out_stop_line,
                           double* out_distance) const;
  const std::vector<Point2D>& referenceLine() const { return reference_; }

 private:
  const MGeoLoader* map_ = nullptr;
  std::vector<std::string> ids_;
  std::vector<Point2D> reference_;
  std::vector<double> cumulative_;
  std::vector<std::string> segment_link_;
  std::vector<StopLineInfo> stops_;
};

}  // namespace behavior_planner
