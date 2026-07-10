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

// planner(frenet_planner_node)가 보낸 외부 궤적을 ReferencePath로 변환한다.
//   ext   : /frenet_planner/trajectory 파싱 결과 (이미 ego 기준 전방 궤적)
//   params: 참고용(현재는 특별한 창/스무딩 파라미터 안 씀)
//   out_ref: 생성된 ReferencePath (출력)
//
// CSV 버전과 달리 closest-point 탐색이나 윈도우 트리밍이 필요 없다 - planner가
// 이미 ego 기준 전방 궤적만 보내주기 때문. v_ref도 재스무딩하지 않는다(이미
// 다항식 최적화로 나온 부드러운 프로파일이라 다시 스무딩하면 왜곡됨).
bool buildReferenceFromExternalTrajectory(
    const ExternalTrajectory& ext,
    const MPCParams&          params,
    ReferencePath&            out_ref);

#endif // PATH_PLANNER_HPP