#ifndef FRENET_AVOIDANCE_SELECTOR_HPP
#define FRENET_AVOIDANCE_SELECTOR_HPP

#include <vector>

#include "frenet_planner/frenet/collision_checker.hpp"
#include "frenet_planner/frenet/ref_line.hpp"
#include "frenet_planner/global/data_logger.hpp"
#include "frenet_planner/global/global.hpp"

// =========================================================
// obstacles 중 "내 차선을 막고 있는 정지/저속 장애물"이 근처에 있으면
// true + 회피 방향/크기(out_offset, lateral_d1 격자에 그대로 더해짐 -
// path_generator.cpp의 ResolveLateralOffset 참고)를 반환한다.
// FSM 미연동 상태의 임시 판단 기준(AvoidConfig 주석 참고).
//
// ProjectObjectToFrenet(leader_selector.hpp)을 재사용한다 - Following과
// 마찬가지로 "장애물 하나를 Frenet으로 투영"하는 로직은 공통.
// =========================================================
bool FindAvoidanceTarget(const RefLine& ref, const FrenetState& ego,
                         const std::vector<ObjectInfo>& obstacles,
                         double lane_width, const VehicleShape& vehicle_shape,
                         const AvoidConfig& cfg, double& out_offset);

#endif
