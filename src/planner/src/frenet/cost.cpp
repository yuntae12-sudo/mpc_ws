#include "frenet/cost.hpp"

double ComputeLateralCost(const FrenetPath& path, const CostWeights& w) {
    const double T  = path.t.empty() ? 0.0 : path.t.back();
    const double d1 = path.d.empty() ? 0.0 : path.d.back();

    return w.kj * path.jerk_cost_lat + w.kt * T + w.kd * d1 * d1;
}

double ComputeLongitudinalCost(const FrenetPath& path, const CostWeights& w) {
    const double T = path.t.empty() ? 0.0 : path.t.back();

    return w.kj * path.jerk_cost_lon + w.kt * T
         + w.ks * path.delta_s * path.delta_s
         + w.ks_dot * path.delta_s_dot * path.delta_s_dot;
}

double ComputeTotalCost(double cost_lat, double cost_lon, const CostWeights& w) {
    return w.klat * cost_lat + w.klon * cost_lon;
}

void EvaluateCosts(std::vector<FrenetPath>& candidates, const CostWeights& w) {
    for (auto& path : candidates) {
        path.cost_lat   = ComputeLateralCost(path, w);
        path.cost_lon   = ComputeLongitudinalCost(path, w);
        path.cost_total = ComputeTotalCost(path.cost_lat, path.cost_lon, w);
    }
}

const FrenetPath* SelectBestPath(const std::vector<FrenetPath>& candidates) {
    const FrenetPath* best = nullptr;

    for (const auto& path : candidates) {
        if (!path.valid) continue;
        if (best == nullptr || path.cost_total < best->cost_total) {
            best = &path;
        }
    }

    return best;
}
