#pragma once
#include <string>
#include <unordered_map>
#include <vector>
#include "behavior_planner/map/link_info.hpp"
#include "behavior_planner/map/stop_line_info.hpp"
namespace behavior_planner {
struct LinkProjection {
  std::string link_id;
  double link_s = 0.0;
  double d = 0.0;
  double distance = 1e9;
};

class MGeoLoader {
 public:
  bool load(const std::string&, const std::string&);
  const std::unordered_map<std::string, LinkInfo>& links() const { return links_; }
  const std::vector<StopLineInfo>& stopLines() const { return stop_lines_; }
  bool getLink(const std::string&, LinkInfo*) const;
  bool findNearestLink(double x, double y, double yaw,
                       LinkProjection* out,
                       double max_distance = 12.0) const;
  bool findForwardDistance(const std::string& from_link_id,
                           double from_link_s,
                           const std::string& to_link_id,
                           double to_link_s,
                           double max_distance,
                           double* out_distance) const;
  bool findNextStopLine(const std::string& current_link_id,
                        double current_link_s,
                        double max_distance,
                        StopLineInfo* out_stop_line,
                        double* out_distance) const;

 private:
  std::unordered_map<std::string, LinkInfo> links_;
  std::unordered_map<std::string, std::vector<std::string>> outgoing_links_;
  std::unordered_map<std::string, std::vector<StopLineInfo>> stop_lines_by_node_;
  std::vector<StopLineInfo> stop_lines_;
};
}
