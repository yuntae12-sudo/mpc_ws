#pragma once

#include <chrono>
#include <string>
#include <vector>

#include "frenet_planner/frenet/path_generator.hpp"
#include "frenet_planner/frenet/ref_line.hpp"
#include "frenet_planner/global/global.hpp"

struct PlannerVisualizationConfig {
    bool enabled = false;
    double publish_hz = 5.0;
    std::string snapshot_path = "/tmp/frenet_planner_debug.json";
};

class PlannerDebugWriter {
public:
    explicit PlannerDebugWriter(PlannerVisualizationConfig config);

    void Publish(const RefLine& ref,
                 const CartesianState& ego,
                 const FrenetState& start,
                 BehaviorState mode,
                 const std::vector<ObjectInfo>& obstacles,
                 const std::vector<FrenetPath>& candidates,
                 int selected_index,
                 const PlannerDebugStats& stats);

private:
    PlannerVisualizationConfig config_;
    std::chrono::steady_clock::time_point last_publish_{};
};
