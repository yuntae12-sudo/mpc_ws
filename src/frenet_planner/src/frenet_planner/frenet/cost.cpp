#include "frenet_planner/frenet/cost.hpp"

double ComputeLateralCost(const FrenetPath& path, const CostWeights& w) {
    const double T  = path.t.empty() ? 0.0 : path.t.back();
    const double d1 = path.d.empty() ? 0.0 : path.d.back();

    // 경로 전체에 걸친 d^2 적분(사다리꼴 적분) - kd*d1^2(끝점만)와 달리 "중간에
    // 얼마나/얼마나 오래 벌어져 있었는지"에 비용을 매긴다(위 kd_path 주석 참고).
    double d_path_integral = 0.0;
    for (size_t i = 0; i + 1 < path.d.size(); ++i) {
        const double dt = path.t[i + 1] - path.t[i];
        d_path_integral += 0.5 * (path.d[i] * path.d[i] + path.d[i + 1] * path.d[i + 1]) * dt;
    }

    return w.kj * path.jerk_cost_lat + w.kt * T + w.kd * d1 * d1
         + w.kd_path * d_path_integral;
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
