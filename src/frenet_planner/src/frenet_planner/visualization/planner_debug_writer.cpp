#include "frenet_planner/visualization/planner_debug_writer.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <utility>

#include "frenet_planner/math/frenet_converter.hpp"

namespace {

const char* ModeName(BehaviorState mode) {
    switch (mode) {
        case LANE_KEEPING: return "LANE_KEEPING";
        case FOLLOWING: return "FOLLOWING";
        case LANE_CHANGE_LEFT: return "LANE_CHANGE_LEFT";
        case LANE_CHANGE_RIGHT: return "LANE_CHANGE_RIGHT";
        case INTERSECTION_WAIT: return "INTERSECTION_WAIT";
        case TURN_LEFT: return "TURN_LEFT";
        case TURN_RIGHT: return "TURN_RIGHT";
        case AVOID: return "AVOID";
        case STOP: return "STOP";
        case EMERGENCY: return "EMERGENCY";
    }
    return "UNKNOWN";
}

const char* ReasonName(RejectionReason reason) {
    switch (reason) {
        case RejectionReason::CURVATURE: return "curvature";
        case RejectionReason::COLLISION: return "collision";
        case RejectionReason::NONE: return "valid";
    }
    return "unknown";
}

void WriteNumber(std::ostream& out, double value) {
    if (std::isfinite(value)) out << value;
    else out << "null";
}

void WriteXY(std::ostream& out, const std::vector<double>& x, const std::vector<double>& y) {
    out << '[';
    const size_t n = std::min(x.size(), y.size());
    for (size_t i = 0; i < n; ++i) {
        if (i) out << ',';
        out << '[';
        WriteNumber(out, x[i]);
        out << ',';
        WriteNumber(out, y[i]);
        out << ']';
    }
    out << ']';
}

}  // namespace

PlannerDebugWriter::PlannerDebugWriter(PlannerVisualizationConfig config)
    : config_(std::move(config)) {}

void PlannerDebugWriter::Publish(const RefLine& ref,
                                 const CartesianState& ego,
                                 const FrenetState& start,
                                 BehaviorState mode,
                                 const std::vector<ObjectInfo>& obstacles,
                                 const std::vector<FrenetPath>& candidates,
                                 int selected_index,
                                 const PlannerDebugStats& stats) {
    if (!config_.enabled || config_.publish_hz <= 0.0) return;

    const auto now = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration<double>(1.0 / config_.publish_hz);
    if (last_publish_.time_since_epoch().count() != 0 && now - last_publish_ < period) return;
    last_publish_ = now;

    std::ostringstream out;
    out << std::setprecision(8);
    out << "{\"mode\":\"" << ModeName(mode) << "\"";
    out << ",\"ego\":{";
    out << "\"x\":"; WriteNumber(out, ego.x);
    out << ",\"y\":"; WriteNumber(out, ego.y);
    out << ",\"yaw\":"; WriteNumber(out, ego.yaw);
    out << ",\"v\":"; WriteNumber(out, ego.v);
    out << ",\"s\":"; WriteNumber(out, start.s);
    out << ",\"d\":"; WriteNumber(out, start.d);
    out << '}';

    out << ",\"stats\":{";
    out << "\"lateral_total\":" << stats.lateral_total;
    out << ",\"lateral_valid\":" << stats.lateral_valid;
    out << ",\"longitudinal_total\":" << stats.longitudinal_total;
    out << ",\"longitudinal_valid\":" << stats.longitudinal_valid;
    out << ",\"combined_total\":" << stats.combined_total;
    out << ",\"after_curvature\":" << stats.combined_valid_after_curvature;
    out << ",\"after_collision\":" << stats.combined_valid_after_collision;
    out << '}';

    out << ",\"obstacles\":[";
    for (size_t i = 0; i < obstacles.size(); ++i) {
        if (i) out << ',';
        const auto& obj = obstacles[i];
        out << "{\"id\":" << obj.id << ",\"type\":" << obj.type;
        out << ",\"x\":"; WriteNumber(out, obj.x);
        out << ",\"y\":"; WriteNumber(out, obj.y);
        out << ",\"heading\":"; WriteNumber(out, obj.heading);
        out << ",\"speed\":"; WriteNumber(out, obj.speed);
        out << ",\"width\":"; WriteNumber(out, obj.width);
        out << ",\"length\":"; WriteNumber(out, obj.length);
        out << '}';
    }
    out << ']';

    out << ",\"candidates\":[";
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (i) out << ',';
        const auto& path = candidates[i];
        GeometricPath geo = ComputeGeometricPath(path.s, path.d, ref);
        out << "{\"index\":" << i;
        out << ",\"status\":\"" << ReasonName(path.rejection_reason) << "\"";
        out << ",\"selected\":" << (static_cast<int>(i) == selected_index ? "true" : "false");
        out << ",\"cost\":"; WriteNumber(out, path.cost_total);
        out << ",\"collision_object_id\":" << path.collision_object_id;
        out << ",\"collision_sample_index\":" << path.collision_sample_index;
        out << ",\"xy\":";
        WriteXY(out, geo.x, geo.y);
        out << '}';
    }
    out << "]}";

    const std::string temp_path = config_.snapshot_path + ".tmp";
    {
        std::ofstream file(temp_path, std::ios::out | std::ios::trunc);
        if (!file.is_open()) return;
        file << out.str();
        if (!file.good()) return;
    }
    if (std::rename(temp_path.c_str(), config_.snapshot_path.c_str()) != 0) {
        std::remove(temp_path.c_str());
    }
}
