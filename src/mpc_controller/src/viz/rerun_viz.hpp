#pragma once

// rerun.io 기반 실시간 시각화 (개발/디버깅 전용).
// ENABLE_RERUN_VIZ가 꺼져 있으면(기본값) 모든 호출이 아무 것도 하지 않는
// inline 함수로 대체되어, 대회 제출 빌드에는 rerun 관련 코드가 전혀 들어가지
// 않는다. CMakeLists.txt의 `ENABLE_RERUN_VIZ` 옵션으로 켠다.

#include <vector>

#include "frenet_planner/frenet/ref_line.hpp"
#include "frenet_planner/global/global.hpp"
#include "frenet_planner/math/frenet_converter.hpp"

#ifdef ENABLE_RERUN_VIZ

namespace rerun_viz {

// 프로그램 시작 시 1회 호출: rerun 뷰어를 spawn하고 recording stream을 연다.
void Init();

// 매 planning 사이클 호출: ego 기준(ego-relative, ego가 원점/+x를 보는 좌표계)으로
// 후보 경로 전체(유효/무효 색 구분)와 선택된 최적 경로를 뷰어에 스트리밍한다.
void LogCycle(const CartesianState& ego,
              const std::vector<FrenetPath>& candidates,
              const FrenetPath* best,
              const RefLine& ref);

}  // namespace rerun_viz

#else

namespace rerun_viz {
inline void Init() {}
inline void LogCycle(const CartesianState&, const std::vector<FrenetPath>&,
                      const FrenetPath*, const RefLine&) {}
}  // namespace rerun_viz

#endif
