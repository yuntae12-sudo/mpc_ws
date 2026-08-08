#ifndef FRENET_AVOIDANCE_SELECTOR_HPP
#define FRENET_AVOIDANCE_SELECTOR_HPP

#include <vector>

#include "frenet_planner/frenet/collision_checker.hpp"
#include "frenet_planner/frenet/ref_line.hpp"
#include "frenet_planner/global/data_logger.hpp"
#include "frenet_planner/global/global.hpp"

struct StaticObstacleTarget {
    int id = -1;
    double s = 0.0;
    double d = 0.0;
    double s_dot = 0.0;
    double width = 0.0;
    double length = 0.0;
    double avoidance_offset = 0.0;
};

// 내 차선 전방의 가장 가까운 정적 장애물과 최초 회피 방향을 반환한다.
// 반환된 ID/방향은 상위 AvoidanceContext가 회피 종료까지 고정한다.
bool FindStaticAvoidanceTarget(const RefLine& ref, const FrenetState& ego,
                               const std::vector<ObjectInfo>& obstacles,
                               double lane_width, const VehicleShape& vehicle_shape,
                               const AvoidConfig& cfg, StaticObstacleTarget& out_target);

#endif
