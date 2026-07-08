#ifndef PLANNER_GLOBAL_BEHAVIOR_BRIDGE_HPP
#define PLANNER_GLOBAL_BEHAVIOR_BRIDGE_HPP

#include <behavior_planner/BehaviorContext.h>
#include <behavior_planner/PlanFeedback.h>

#include "global/global.hpp"
#include "frenet/cost.hpp"

// =========================================================
// /behavior/context <-> planner 연동 (INTEGRATION_PLAN.md 1~2번).
// behavior_planner_frenet_handoff_guide.pdf 4.1~4.3절이 정의한 계약을
// 그대로 따름 — front_gap/front_relative_speed 등 leader 운동학은
// "Frenet 이 반드시 읽을 값" 목록에 없어(그 문서 4.2절) 여기서 안 씀.
// =========================================================

struct BehaviorBridgeConfig {
    double lane_width = 3.5;              // [m] 차선변경 목표 d 오프셋
    double context_timeout = 0.5;         // [s] BehaviorContext staleness 임계치
    double emergency_stop_buffer = 12.0;  // [m] stop_before_s sentinel(1e9)일 때 대신 쓸 거리
    CostWeights emergency_cost_weights{};
};

// BehaviorContext -> PlannerCommand. enable_*/forbid_* 방어적 검증 포함
// (문서 4.2절: "enable=false 후보는 만들거나 선택하지 않는 것이 안전함").
PlannerCommand BuildCommandFromContext(const behavior_planner::BehaviorContext& ctx,
                                        const FrenetState& ego_start,
                                        const BehaviorBridgeConfig& cfg);

// 이번 사이클 결과 -> PlanFeedback. behavior_planner가 지금 이 값을
// BehaviorDecision에 반영하지 않는다고 문서에 명시돼 있어(4.3절 "통합
// 주의점"), 필드 정밀도보다 스키마를 끊기지 않게 채우는 것을 우선함.
behavior_planner::PlanFeedback BuildFeedback(const PlannerCommand& cmd_used,
                                              const FrenetPath* best,
                                              const behavior_planner::BehaviorContext& ctx);

#endif
