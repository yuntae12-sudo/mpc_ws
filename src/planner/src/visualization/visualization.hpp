#ifndef PLANNER_VISUALIZATION_HPP
#define PLANNER_VISUALIZATION_HPP

#include <string>
#include <vector>

#include <visualization_msgs/MarkerArray.h>

#include "frenet/ref_line.hpp"
#include "frenet/collision_checker.hpp"
#include "math/frenet_converter.hpp"
#include "global/global.hpp"

// =========================================================
// rviz 시각화 — 논문 Fig.3~6 스타일(최적 경로=초록, 유효 후보=회색,
// 무효 후보=옅은 빨강)로 후보 궤적 전체를 MarkerArray로 그린다.
// planner 알고리즘 자체(path_generator/cost/collision_checker)는 이 파일에
// 전혀 의존하지 않는다 — main.cpp가 계산 결과를 넘겨주면 그걸 그림으로
// 바꾸는 역할만 한다 (관심사 분리).
// =========================================================

// 후보 궤적 전체 (best=초록/굵게, valid=회색/얇게, invalid=빨강/반투명).
// candidates는 FilterByCurvature/FilterByCollision까지 거친 뒤 상태를 그대로 사용.
visualization_msgs::MarkerArray BuildCandidateMarkers(const std::vector<FrenetPath>& candidates,
                                                       const RefLine& ref,
                                                       const FrenetPath* best,
                                                       const std::string& frame_id);

// 센터라인(RefLine) 중 자차 근방(center_s ± window)만 얇은 파란 선으로 표시.
// 트랙 전체(2.6km)를 매 사이클 다 그리면 낭비라 근방만 그린다.
visualization_msgs::Marker BuildRefLineMarker(const RefLine& ref, double center_s, double window,
                                               const std::string& frame_id, int id);

// 전역 경로(RefLine) 전체를 흰색 선으로 표시. 경로 자체는 주행 중 안 바뀌므로
// 이건 매 사이클이 아니라 노드 시작 시 한 번만 만들어서 latched 토픽으로
// 발행하면 된다 (BuildRefLineMarker의 "근방만" 버전과 별개 — 전체 조망용).
visualization_msgs::Marker BuildGlobalPathMarker(const RefLine& ref, const std::string& frame_id, int id);

// 자차를 실제 크기(VehicleShape)의 노란 박스로 표시.
visualization_msgs::Marker BuildEgoMarker(const CartesianState& cs, const VehicleShape& shape,
                                           const std::string& frame_id, int id);

// 장애물들을 실제 크기의 빨간 박스로 표시.
visualization_msgs::MarkerArray BuildObstacleMarkers(const std::vector<ObjectInfo>& obstacles,
                                                      const std::string& frame_id);

// =========================================================
// map -> ego_frame tf를 매 사이클 발행한다. 이 프로젝트엔 원래 tf 트리가
// 없는데(GPS-ENU 좌표를 직접 씀), rviz 카메라가 자차를 자동으로 따라가려면
// (Views 패널의 카메라 Target Frame 기능) 추적할 tf 프레임이 필요해서 추가함.
// 카메라 추적 "전용"이라 planner 알고리즘(Frenet 변환 등)은 이 tf를 전혀
// 참조하지 않는다 — 순수 시각화 편의 기능.
// =========================================================
void BroadcastEgoTransform(const CartesianState& cs,
                            const std::string& map_frame,
                            const std::string& ego_frame);

#endif
