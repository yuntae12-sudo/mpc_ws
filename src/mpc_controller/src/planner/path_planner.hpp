#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include "../global/global.hpp"

// ========================================
// Path Planner
//   현재 ego 위치 기준으로 waypoints에서
//   ReferencePath를 생성하는 Planner 역할
//
//   나중에 Expert / PA / SA로 교체할 때
//   이 파일만 바꾸면 됨
// ========================================

// path.txt 기반 reference path 생성
//   ego_snap : 현재 차량 상태
//   waypoints: 전체 경로 (CSV에서 로드된 것)
//   params   : MPC 파라미터 (ref_window, 속도 임계값 등)
//   out_ref  : 생성된 ReferencePath (출력)
//   closest_idx: 가장 가까운 waypoint 인덱스 캐시 (입출력)
//
// 반환값: reference path 생성 성공 여부
bool buildReferenceFromWaypoints(
    const MPCState&          ego_snap,
    const std::vector<Waypoint>& waypoints,
    const MPCParams&         params,
    ReferencePath&           out_ref,
    int&                     closest_idx);

#endif // PATH_PLANNER_HPP