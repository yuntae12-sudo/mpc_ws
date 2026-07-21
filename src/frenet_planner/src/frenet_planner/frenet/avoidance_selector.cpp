#include "frenet_planner/frenet/avoidance_selector.hpp"

#include <cmath>
#include <limits>

#include "frenet_planner/frenet/leader_selector.hpp"

namespace {

// candidate_offset 쪽으로 피했을 때, trigger_distance 안의 "다른" 장애물과
// 겹치지 않는지 확인. 자차 폭/장애물 폭을 반영한 최소 이격 거리 + 여유 마진.
bool IsSideClear(const RefLine& ref, const FrenetState& ego,
                 const std::vector<ObjectInfo>& obstacles,
                 double candidate_offset, const VehicleShape& vehicle_shape,
                 double trigger_distance) {
    constexpr double kMargin = 0.3;  // [m] 폭 계산 여유

    for (const auto& obj : obstacles) {
        double s, d, s_dot;
        ProjectObjectToFrenet(ref, obj, s, d, s_dot);

        const double gap = s - ego.s;
        if (gap <= 0.0 || gap > trigger_distance) continue;

        const double need = vehicle_shape.width * 0.5 + obj.width * 0.5 + kMargin;
        if (std::fabs(d - candidate_offset) < need) return false;
    }
    return true;
}

}  // namespace

bool FindAvoidanceTarget(const RefLine& ref, const FrenetState& ego,
                         const std::vector<ObjectInfo>& obstacles,
                         double lane_width, const VehicleShape& vehicle_shape,
                         const AvoidConfig& cfg, double& out_offset) {
    const double half_lane = lane_width * 0.5;

    // 1. 내 차선을 막고 있는 정지/저속 장애물 중 가장 가까운 것을 찾는다.
    bool found = false;
    double best_gap = std::numeric_limits<double>::max();

    for (const auto& obj : obstacles) {
        if (obj.speed > cfg.trigger_max_speed) continue;  // Following이 처리할 movtng 장애물

        double s, d, s_dot;
        ProjectObjectToFrenet(ref, obj, s, d, s_dot);

        const double gap = s - ego.s;
        if (gap <= 0.0 || gap > cfg.trigger_distance) continue;
        if (std::fabs(d) > half_lane) continue;  // 내 차선 밖은 안 막고 있음

        if (gap < best_gap) {
            best_gap = gap;
            found = true;
        }
    }

    if (!found) return false;

    // 2. 왼쪽(+)/오른쪽(-) 중 다른 장애물과 안 겹치는 쪽을 고른다. 둘 다
    // 막혀있으면(드문 경우) best-effort로 왼쪽을 반환 - 최종 안전은
    // FilterByCollision(SAT)이 그 오프셋 후보들까지 다시 검사해서 보장한다
    // (다 막혀있으면 valid 후보가 0개가 되어 상위(main.cpp)의 정지 폴백으로
    // 안전하게 넘어감).
    const bool left_clear  = IsSideClear(ref, ego, obstacles,  cfg.avoid_offset, vehicle_shape, cfg.trigger_distance);
    const bool right_clear = IsSideClear(ref, ego, obstacles, -cfg.avoid_offset, vehicle_shape, cfg.trigger_distance);

    if (left_clear) {
        out_offset = cfg.avoid_offset;
    } else if (right_clear) {
        out_offset = -cfg.avoid_offset;
    } else {
        out_offset = cfg.avoid_offset;
    }
    return true;
}
