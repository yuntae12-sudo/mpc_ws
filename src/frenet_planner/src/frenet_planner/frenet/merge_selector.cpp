#include "frenet_planner/frenet/merge_selector.hpp"

#include <algorithm>
#include <cmath>

namespace {
struct Arrival { int id; double time; };

// 현재 속도에서 accel로 desired_speed까지 가속한 뒤 정속 주행할 때의
// 최소 도달시간. MERGE gap 판정과 실제 종방향 후보가 서로 다른 속도를
// 가정하지 않도록 정지 상태에서도 가속 시간을 명시적으로 포함한다.
double MinimumTravelTime(double distance, double initial_speed,
                         double desired_speed, double accel) {
    if (distance <= 0.0) return 0.0;
    const double v0 = std::max(0.0, initial_speed);
    const double target = std::max(desired_speed, 0.5);
    const double a = std::max(accel, 0.1);
    if (v0 >= target) return distance / std::max(v0, 0.5);

    const double accel_time = (target - v0) / a;
    const double accel_distance = v0 * accel_time + 0.5 * a * accel_time * accel_time;
    if (distance <= accel_distance) {
        return (-v0 + std::sqrt(v0 * v0 + 2.0 * a * distance)) / a;
    }
    return accel_time + (distance - accel_distance) / target;
}
}

RoundaboutGap FindRoundaboutGap(const RefLine& ref, const FrenetState& ego,
                                const std::vector<ObjectInfo>& obstacles,
                                const MergeConfig& cfg, double desired_speed,
                                double max_ego_accel,
                                const std::unordered_map<int, double>* yaw_rates) {
    RoundaboutGap result;
    if (cfg.conflict_s < 0.0 || ref.points.empty()) return result;

    const RefPoint conflict = Interpolate(ref, cfg.conflict_s);
    std::vector<Arrival> arrivals;
    for (const auto& obj : obstacles) {
        if (obj.speed < cfg.min_object_speed) continue;
        double yaw_rate = obj.yaw_rate;
        if (yaw_rates) {
            const auto it = yaw_rates->find(obj.id);
            if (it != yaw_rates->end()) yaw_rate = it->second;
        }

        // 회전교차로 차량을 현재 heading의 직선으로만 외삽하면 접선 방향 때문에
        // conflict point를 보지 못한다. 관측된 yaw-rate를 이용한 constant-turn-rate
        // 궤적을 샘플링해 conflict point 최근접 도달시각을 구한다.
        const double t_end = cfg.max_wait_time + cfg.ego_clear_time;
        constexpr double kPredictionDt = 0.05;
        double best_t = 0.0;
        double best_miss = std::hypot(conflict.x - obj.x, conflict.y - obj.y);
        for (double t = kPredictionDt; t <= t_end + 1e-9; t += kPredictionDt) {
            double px, py;
            if (std::fabs(yaw_rate) > 1e-3) {
                px = obj.x + obj.speed / yaw_rate *
                    (std::sin(obj.heading + yaw_rate * t) - std::sin(obj.heading));
                py = obj.y - obj.speed / yaw_rate *
                    (std::cos(obj.heading + yaw_rate * t) - std::cos(obj.heading));
            } else {
                px = obj.x + obj.speed * std::cos(obj.heading) * t;
                py = obj.y + obj.speed * std::sin(obj.heading) * t;
            }
            const double miss = std::hypot(conflict.x - px, conflict.y - py);
            if (miss < best_miss) {
                best_miss = miss;
                best_t = t;
            }
        }
        if (best_miss > cfg.conflict_radius) continue;
        arrivals.push_back({obj.id, best_t});
    }
    std::sort(arrivals.begin(), arrivals.end(),
              [](const Arrival& a, const Arrival& b) { return a.time < b.time; });
    result.crossing_vehicle_count = arrivals.size();

    const double distance = std::max(0.0, cfg.conflict_s - ego.s);
    const double earliest = MinimumTravelTime(
        distance, ego.s_d, desired_speed, max_ego_accel);
    // rear gap은 고정 ego_clear_time이 아니라, 현재 가속 능력으로 conflict
    // point부터 completion_distance까지 실제로 빠져나가는 시간을 사용한다.
    // 설정값은 측정/모델 오차에 대한 최소 하한으로만 유지한다.
    const double entry_speed = std::min(
        std::max(desired_speed, 0.5),
        std::sqrt(std::max(0.0, ego.s_d * ego.s_d +
                                   2.0 * std::max(max_ego_accel, 0.1) * distance)));
    const double clear_duration = std::max(
        cfg.ego_clear_time,
        MinimumTravelTime(cfg.completion_distance, entry_speed,
                          desired_speed, max_ego_accel));
    result.clear_time = clear_duration;

    // Ego가 너무 멀어 도착 자체가 예측 시간범위 밖이면 현재 관측 차량으로
    // gap을 확정할 단계가 아니다. 이를 unsafe로 반환하면 정지 상태에서
    // earliest>max_wait가 영원히 유지되어 출발조차 못 하는 자기잠금이 된다.
    // 이 구간은 global path로 접근하고, max_wait 범위에 들어온 뒤 매 cycle
    // 실제 순환 차량을 다시 관측해 WAIT/ENTER를 결정한다.
    if (earliest > cfg.max_wait_time) {
        // 안전하다는 뜻이 아니라 아직 Ego 도착시각이 예측창 밖이라는 뜻이다.
        // APPROACH는 계속할 수 있지만 COMMIT 근거로 사용할 수 없다.
        result.confirmed = false;
        result.entry_time = earliest;
        return result;
    }
    result.confirmed = true;

    constexpr double kSearchDt = 0.1;
    for (double entry = earliest; entry <= cfg.max_wait_time + 1e-9; entry += kSearchDt) {
        int preceding_id = -1, following_id = -1;
        double preceding_time = -1.0, following_time = -1.0;
        for (const auto& arrival : arrivals) {
            if (arrival.time <= entry) {
                preceding_id = arrival.id;
                preceding_time = arrival.time;
            } else {
                following_id = arrival.id;
                following_time = arrival.time;
                break;
            }
        }
        const bool front_clear = preceding_id < 0 ||
            entry - preceding_time >= cfg.min_front_time_gap;
        const bool rear_clear = following_id < 0 ||
            following_time - entry >= clear_duration + cfg.min_rear_time_gap;
        if (front_clear && rear_clear) {
            result.safe = true;
            result.entry_time = entry;
            result.preceding_id = preceding_id;
            result.preceding_time = preceding_time;
            result.following_id = following_id;
            result.following_time = following_time;
            return result;
        }
    }
    result.entry_time = cfg.max_wait_time;
    return result;
}
