#pragma once

#include <string>

#include "frenet_planner/frenet/ref_line.hpp"
#include "frenet_planner/frenet/path_generator.hpp"
#include "frenet_planner/frenet/cost.hpp"
#include "frenet_planner/frenet/collision_checker.hpp"
#include "frenet_planner/visualization/planner_debug_writer.hpp"

// =========================================================
// 곡률 기반 사전 감속 (mpc_controller의 옛 planner/path_planner.cpp 로직을
// Frenet Frame Planner 쪽으로 이식한 것). lookahead 구간 내 최대 곡률로
// velocity-keeping 목표속도를 정한다.
// =========================================================
struct CurveSpeedConfig {
    double target_vel        = 10.0;
    double curve_vel_sharp   = 4.0;
    double curve_vel_mid     = 5.5;
    double curve_vel_mild    = 7.0;
    double curve_th_sharp    = 0.080;
    double curve_th_mid      = 0.035;
    double curve_th_mild     = 0.015;
    double curve_lookahead_m = 15.0;
};

// =========================================================
// Following(Sec.V-A) 모드 파라미터. leader_selector.cpp가 obstacles에서
// 선두 차량을 고를 때(탐색 범위)와 GenerateFollowingCandidates가
// constant-time-gap-law target을 만들 때(time_gap=tau, min_gap=D0) 쓴다.
// =========================================================
struct FollowingConfig {
    double time_gap = 1.0;              // [s] tau
    double min_gap  = 5.0;              // [m] D0
    double gap_gain = 0.3;              // [1/s] v_cmd = v_leader + gain * gap_error
    double max_leader_search_s = 60.0;  // [m] 이 거리 안의 선두 차량만 후보로 봄
    double min_leader_speed = 0.5;      // [m/s] 이보다 느리면 정적 장애물로 보고 FOLLOWING에서 제외
    double exit_search_margin = 10.0;   // [m] 진입 후 탐색거리 hysteresis
    int dropout_grace_cycles = 5;       // [cycle] ObjectInfo 일시 누락 허용(20Hz)
};

// =========================================================
// AVOID 트리거/오프셋 결정용 파라미터. FSM 미연동 상태의 임시 판단 기준 -
// "내 차선을 막고 있는 정지/저속 장애물이 가까이 있으면 피한다"만 반영.
// (behavior_planner 연동 시 진짜 트리거 조건으로 교체될 자리표시자)
// =========================================================
struct AvoidConfig {
    double detection_distance = 60.0; // [m] 최소 등록거리(실제값은 충돌 lookahead와 자동 조정)
    double shift_start_distance = 25.0; // [m] 횟이동을 시작할 장애물과의 종방향 거리
    double trigger_max_speed = 0.5;   // [m/s] 이 속도 이하만 "정지 장애물"로 간주(그 이상은 Following이 처리)
    // [m] 회피 시 lateral_d1 격자를 밀어낼 크기. 자차 폭(1.9m 기준) + 장애물
    // 폭 절반씩 + 여유(0.3m)를 감안한 최소 이격(~2.2m)보다 확실히 크게 잡아야
    // avoidance_selector.cpp의 좌/우 클리어 판정이 "장애물 자신" 때문에 항상
    // 막힌 것으로 나오지 않는다.
    double avoid_offset      = 3.0;
    double comfortable_decel = 3.0;   // [m/s^2] 추후 동적 시작거리 계산용(현재 미사용)
    double lateral_tolerance = 0.4;   // [m] SHIFT 완료 판정
    double pass_clearance    = 1.0;   // [m] 자차 뒤가 장애물을 통과한 뒤 추가 여유
    bool prefer_right_when_equal = true; // 좌/우 후보 품질이 같을 때 우측 우선
};

// waypoint 파일("x y" 또는 "x,y") -> RefLine
bool LoadReferenceLine(const std::string& path, RefLine& out_ref, double max_curvature);

// params.yaml -> 각 config struct 로드 (yaml-cpp 기반, ROS 의존성 없음)
void LoadParams(const std::string& yaml_path,
                PathGeneratorConfig& path_cfg,
                KinematicLimits& limits,
                CostWeights& cost_weights,
                VehicleShape& vehicle_shape,
                CollisionCheckConfig& collision_cfg,
                CurveSpeedConfig& curve_speed_cfg,
                FollowingConfig& following_cfg,
                AvoidConfig& avoid_cfg,
                PlannerVisualizationConfig& visualization_cfg,
                double& wheelbase,
                double& lane_width,
                std::string& waypoint_file);
