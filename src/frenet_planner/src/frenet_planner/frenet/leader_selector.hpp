#ifndef FRENET_LEADER_SELECTOR_HPP
#define FRENET_LEADER_SELECTOR_HPP

#include <vector>

#include "frenet_planner/frenet/ref_line.hpp"
#include "frenet_planner/global/data_logger.hpp"
#include "frenet_planner/global/global.hpp"

// =========================================================
// obstacles(ObjectInfo) -> Frenet 투영/선두 차량 선택.
// Following 전용으로 만들었지만, "장애물 하나를 Frenet으로 투영한다"는
// ProjectObjectToFrenet 자체는 AVOID/Merge에서도 그대로 재사용 가능
// (계획서 참고: 전방/후방 두 차량을 다뤄야 하는 Merge에서 이걸 두 번 씀).
// =========================================================

// 장애물 하나를 참조선 기준 Frenet 좌표로 투영한다.
// out_s: 장애물의 s(참조선 arc length), out_d: 장애물의 lateral offset,
// out_s_dot: 참조선 진행 방향으로 투영한 장애물의 종방향 속도(식(7) 역산 -
// 장애물 heading이 참조선과 어긋나 있으면 raw speed와 다를 수 있음).
// 탐색 범위 제한 없이 항상 전체 참조선에서 최근접점을 찾는다(장애물은
// ego처럼 매 사이클 위치가 조금씩만 바뀐다는 연속성 가정을 할 수 없고,
// Following 탐색 거리(수십 m)가 FindClosestS의 기본 hint window(30m)보다
// 클 수 있어 hint를 안 쓰는 게 안전하다).
void ProjectObjectToFrenet(const RefLine& ref, const ObjectInfo& obj,
                           double& out_s, double& out_d, double& out_s_dot);

// ego 앞, 내 차선 안(|d| <= lane_width/2)에서 가장 가까운 차량을 선두로 고른다.
// cfg.max_leader_search_s보다 먼 차량은 후보에서 제외.
// 선두를 찾으면 true + out_leader_*를 채우고, 없으면 false.
// out_leader_accel은 항상 0.0 - ObjectInfo에 가속도 정보가 없어 등속(상수
// 속도) 가정으로 둔다(GenerateFollowingCandidates도 "등가속 예측"에서
// accel=0이면 등속 예측으로 자연히 축소됨, 별도 처리 불필요).
bool FindLeader(const RefLine& ref, const FrenetState& ego,
                const std::vector<ObjectInfo>& obstacles,
                double lane_width, const FollowingConfig& cfg,
                double& out_leader_s, double& out_leader_speed, double& out_leader_accel);

#endif
