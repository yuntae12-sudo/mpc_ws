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

namespace {

double MarginFraction(BehaviorState mode, const HysteresisConfig& cfg) {
    switch (mode) {
        case EMERGENCY:
        case AVOID:
            return cfg.margin_fraction_emergency;
        case STOP:
        case INTERSECTION_WAIT:
            return cfg.margin_fraction_stopping;
        default:
            return cfg.margin_fraction_velocity_keeping;
    }
}

// last_best와의 "격자 몇 칸 차이인지"를 각 SamplingRange.step으로 정규화해서 구한다
// (0.5m 격자와 1.0m/s 격자가 raw 값으로 섞이면 한쪽 차원이 부당하게 우세해짐).
double GridDistance(const FrenetPath& path, const LastBestParams& last, const PathGeneratorConfig& cfg) {
    const double d1 = path.d.empty() ? 0.0 : path.d.back();
    const double T = path.t.empty() ? 0.0 : path.t.back();

    const double dd1 = (d1 - last.d1) / cfg.lateral_d1.step;
    const double dT = (T - last.T) / cfg.time_horizon.step;

    double dparam_sq = 0.0;
    if (cfg.delta_s.step > 0.0) {
        const double dds = (path.delta_s - last.delta_s) / cfg.delta_s.step;
        dparam_sq += dds * dds;
    }
    if (cfg.delta_s_dot.step > 0.0) {
        const double ddsd = (path.delta_s_dot - last.delta_s_dot) / cfg.delta_s_dot.step;
        dparam_sq += ddsd * ddsd;
    }

    return std::sqrt(dd1 * dd1 + dT * dT + dparam_sq);
}

const FrenetPath* FindClosestToLast(const std::vector<FrenetPath>& candidates,
                                     const LastBestParams& last,
                                     const PathGeneratorConfig& cfg) {
    const FrenetPath* closest = nullptr;
    double closest_dist = std::numeric_limits<double>::max();
    for (const auto& path : candidates) {
        if (!path.valid) continue;
        const double dist = GridDistance(path, last, cfg);
        if (dist < closest_dist) {
            closest_dist = dist;
            closest = &path;
        }
    }
    return closest;
}

}  // namespace

const FrenetPath* SelectBestPathWithHysteresis(const std::vector<FrenetPath>& candidates,
                                                BehaviorState mode,
                                                const PathGeneratorConfig& path_cfg,
                                                const HysteresisConfig& hyst_cfg,
                                                LastBestParams& last_best) {
    const FrenetPath* best_this_cycle = SelectBestPath(candidates);
    if (!best_this_cycle) {
        last_best.valid = false;
        return nullptr;
    }

    const double margin = MarginFraction(mode, hyst_cfg) * best_this_cycle->cost_total;
    const FrenetPath* chosen = best_this_cycle;

    if (last_best.valid && last_best.mode == mode) {
        const FrenetPath* closest = FindClosestToLast(candidates, last_best, path_cfg);
        if (closest && closest->cost_total <= best_this_cycle->cost_total + margin) {
            chosen = closest;
        }
    }

    last_best.valid = true;
    last_best.mode = mode;
    last_best.d1 = chosen->d.empty() ? 0.0 : chosen->d.back();
    last_best.T = chosen->t.empty() ? 0.0 : chosen->t.back();
    last_best.delta_s = chosen->delta_s;
    last_best.delta_s_dot = chosen->delta_s_dot;

    return chosen;
}
