// 기본적인 헤더 파일 및 FSM에서 사용하는 입,출력 구조체 들어가는 곳

#ifndef PLANNER_GLOBAL_HPP
#define PLANNER_GLOBAL_HPP

#include <vector>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "fsm/state.hpp"

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

// ---- FSM 입력 구조체 (data_logger -> fsm) -----------
struct FsmInput {
    // 1. 자차 상태값
    double ego_x;
    double ego_y;
    double ego_heading;
    double ego_speed;
    double ego_accel;
    double ego_lateral_offset;

    // 2. 주변 오브젝트
    std::vector<ObjectInfo> npc_list;
    std::vector<ObjectInfo> pedestrian_list;
    std::vector<ObjectInfo> obstacle_list;

    // 3. 충돌 감지
    bool is_collision;

    // 4. 신호등/정지선 (토픽 아직 몰라서 안적음, 추후 추가해야 함)
    // int traffic_light_state;        // (0:RED, 1:YELLOW, 2:GRREN)
    // double stop_line_distance;      // 정지선까지의 거리 [m]
    // bool has_crosswalk;             // 횡단보도 존재 여부
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
