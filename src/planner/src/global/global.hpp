// 기본적인 헤더 파일 및 planner 전역 입,출력 구조체 들어가는 곳

#ifndef PLANNER_GLOBAL_HPP
#define PLANNER_GLOBAL_HPP

#include <vector>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "global/state.hpp"

// --- ObjectStatus 한 오브젝트 -------------
struct ObjectInfo {
    int id;
    int type;
    double x;
    double y;
    double heading;
    double speed;
    double width;
    double length;
};

struct PlannerCommand {
    // 1. 모드
    BehaviorState mode;

    // 2. 횡방향 목표
    int target_lane;                   // -1: 좌, 0: 유지, 1: 우
    double avoidance_d_offset;         // AVOID 모드 전용 연속값 [m]

    // 3. 종방향 목표
    double target_speed;               // Velocity_keep, Lane_change [m/s]
    double stop_position;              // STOP 모드: 정지 목표 s 좌표 [m]

    // 4. FOLLOWING 전용
    double leader_s;                   // 선두 차량 위치 [m]
    double leader_speed;               // 선두 차량 속도 [m/s]
    double leader_accel;               // 선두 차량 가속도
    double time_gap;                   // taw
    double min_gap;                    // D0 [m]
};

#endif
