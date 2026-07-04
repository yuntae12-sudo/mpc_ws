// 현재 상태들에 대해 정의하는 파일 (어떤 주행을 해야하는지 ex: Lane_tracking, ACC 등)
#ifndef PLANNER_FSM_STATE_HPP
#define PLANNER_FSM_STATE_HPP

enum BehaviorState {
    LANE_KEEPING,
    FOLLOWING,
    LANE_CHANGE_LEFT,
    LANE_CHANGE_RIGHT,
    INTERSECTION_WAIT,
    TURN_LEFT,
    TURN_RIGHT,
    AVOID,
    STOP,
    EMERGENCY
};

#endif
