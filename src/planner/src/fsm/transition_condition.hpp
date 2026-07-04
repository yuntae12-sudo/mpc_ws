// 전이 조건 판단 함수 선언
// DecideState(현재 상태, 입력) -> 다음 상태 반환

#ifndef PLANNER_FSM_TRANSITION_CONDITION_HPP
#define PLANNER_FSM_TRANSITION_CONDITION_HPP

#include "global/global.hpp"

BehaviorState DecideState(BehaviorState current_state, const FsmInput& input);

#endif
