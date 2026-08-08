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

bool FindStaticAvoidanceTarget(const RefLine& ref, const FrenetState& ego,
                               const std::vector<ObjectInfo>& obstacles,
                               double lane_width, const VehicleShape& vehicle_shape,
                               const AvoidConfig& cfg, StaticObstacleTarget& out_target) {
    const double half_lane = lane_width * 0.5;

    // 1. 내 차선을 막고 있는 정지/저속 장애물 중 가장 가까운 것을 찾는다.
    bool found = false;
    double best_gap = std::numeric_limits<double>::max();
    const ObjectInfo* best_obj = nullptr;
    double best_s = 0.0, best_d = 0.0, best_s_dot = 0.0;

    for (const auto& obj : obstacles) {
        double s, d, s_dot;
        ProjectObjectToFrenet(ref, obj, s, d, s_dot);

        // raw obj.speed(크기만 있는 속력)가 아니라 경로에 투영한 종방향
        // 속도(s_dot)로 "정지했는지"를 판단한다. 보행자처럼 내 진행 방향과
        // 수직으로 움직이는 장애물은 raw speed는 0이 아니지만 s_dot은
        // cos(delta_theta)~0이라 거의 0이 된다 - 그래야 "내 경로 기준으로는
        // 정지해 있다"는 실제 의도가 맞게 반영된다. 회전교차로처럼 차량이
        // 선회하며 접근하는 경우도 마찬가지로 raw speed와 s_dot이 갈릴 수
        // 있어 경로 기준 값을 써야 한다.
        if (std::fabs(s_dot) > cfg.trigger_max_speed) {
            // TODO(dynamic avoidance): 상대속도/TTC/미래 위치를 이용한 동적
            // 장애물 회피를 같은 AVOID 모드의 별도 context로 추가할 것.
            continue;
        }

        const double gap = s - ego.s;
        if (gap <= 0.0 || gap > cfg.detection_distance) continue;
        if (std::fabs(d) > half_lane) continue;  // 내 차선 밖은 안 막고 있음

        if (gap < best_gap) {
            best_gap = gap;
            found = true;
            best_obj = &obj;
            best_s = s;
            best_d = d;
            best_s_dot = s_dot;
        }
    }

    if (!found) return false;

    // 2. 왼쪽(+)/오른쪽(-) 중 다른 장애물과 안 겹치는 쪽을 고른다. 둘 다
    // 막혀있으면(드문 경우) best-effort로 왼쪽을 반환 - 최종 안전은
    // FilterByCollision(SAT)이 그 오프셋 후보들까지 다시 검사해서 보장한다
    // (다 막혀있으면 valid 후보가 0개가 되어 상위(main.cpp)의 정지 폴백으로
    // 안전하게 넘어감).
    const bool left_clear  = IsSideClear(ref, ego, obstacles,  cfg.avoid_offset, vehicle_shape, cfg.detection_distance);
    const bool right_clear = IsSideClear(ref, ego, obstacles, -cfg.avoid_offset, vehicle_shape, cfg.detection_distance);

    if (left_clear) {
        out_target.avoidance_offset = cfg.avoid_offset;
    } else if (right_clear) {
        out_target.avoidance_offset = -cfg.avoid_offset;
    } else {
        out_target.avoidance_offset = cfg.avoid_offset;
    }
    out_target.id = best_obj->id;
    out_target.s = best_s;
    out_target.d = best_d;
    out_target.s_dot = best_s_dot;
    out_target.width = best_obj->width;
    out_target.length = best_obj->length;
    return true;
}
