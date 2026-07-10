#ifndef PLANNER_GLOBAL_BEHAVIOR_BRIDGE_HPP
#define PLANNER_GLOBAL_BEHAVIOR_BRIDGE_HPP

#include <behavior_planner/BehaviorContext.h>
#include <behavior_planner/PlanFeedback.h>

#include "global/global.hpp"
#include "frenet/cost.hpp"
#include "math/frenet_converter.hpp"

// =========================================================
// /behavior/context <-> planner 연동 (INTEGRATION_PLAN.md 1~2번).
// behavior_planner_frenet_handoff_guide.pdf 4.1~4.3절이 정의한 계약을
// 그대로 따름 — front_gap/front_relative_speed 등 leader 운동학은
// "Frenet 이 반드시 읽을 값" 목록에 없어(그 문서 4.2절) 여기서 안 씀.
// =========================================================

struct BehaviorBridgeConfig {
    double lane_width = 3.5;              // [m] 차선변경 목표 d 오프셋
    double context_timeout = 0.5;         // [s] BehaviorContext staleness 임계치
    CostWeights emergency_cost_weights{};

    // EMERGENCY인데 ctx.stop_before_s가 sentinel(behavior_planner의 hard_rule_filter.cpp
    // 확인 결과, TTC/emergency_risk로 트리거된 EMERGENCY는 stop_before_s를 아예 설정하지
    // 않고 sentinel(1e9) 그대로 둠 - FSM은 "멈춰야 한다"는 판단만 하고 "물리적으로 어디서
    // 멈출 수 있는가"는 계산하지 않는다)일 때, planner가 직접 물리 기반 정지거리를 계산한다
    // (behavior_bridge.cpp 참고). 예전엔 고정 12m 버퍼를 썼는데, 고속(예: 33.3m/s)에서
    // 12m 안에 멈추려면 46m/s² 감속이 필요해 max_longitudinal_accel(3.0)을 훨씬 넘어서
    // FilterLongitudinalByAcceleration이 STOP 후보를 전부 걸러내는 버그가 있었다
    // (위급 상황일수록 후보가 전멸하는 역설 - 실차 검증으로 확인).
    double emergency_decel_margin = 0.5;   // stop_distance = v²/(2*max_longitudinal_accel*이 값)
                                            // - quintic의 첨두감속도가 평균보다 큰 만큼 여유를 둠
    double emergency_reaction_buffer = 3.0;  // [m] 계산된 물리적 정지거리 위에 더할 반응거리 여유
    double emergency_max_decel = 3.0;      // [m/s^2] main.cpp에서 g_limits.max_longitudinal_accel로
                                            // 채워짐 (단일 소스 유지, 별도 yaml 키 없음)

    // 위 물리 공식은 "얼마나 가야 멈출 수 있는가"만 계산해서 실제 장애물
    // 위치를 모른다 - 계산된 stop_position이 장애물을 넘어서면 모든 STOP
    // 후보가 장애물을 관통해야 해서 FilterByCollision이 전부 걸러내는
    // 역설이 있었다(실차 검증, 2026-07-08). 그래서 자차 앞쪽 장애물의 s를
    // 직접 찾아 stop_position이 그 앞에서 멈추도록 clamp한다. 이 마진은
    // vehicle_shape.length/2(≈2.25) + 안전여유를 반올림한 값.
    double emergency_obstacle_margin = 3.0;  // [m]

    // ctx.desired_speed는 링크가 바뀌는 순간 계단식으로 크게 뛴다(예: 8.0->33.3).
    // 이걸 target_speed로 그대로 꽂으면 velocity-keeping 후보(quartic, 종료
    // 가속도=0 조건)가 짧은 horizon 안에 그 격차를 메우기 위해 평균의 약 2배에
    // 달하는 첨두 가속도를 요구하게 되고, max_longitudinal_accel을 넘어 후보가
    // 전멸하는 버그가 있었다(실차 검증으로 확인 - 정지 출발 불가, 고속도로 버그
    // 등 여러 증상으로 나타남). desired_speed를 직접 목표로 쓰는 대신, 이 가속도
    // (안전한계 max_longitudinal_accel보다 훨씬 낮은 "승차감용" 값)로 서서히
    // 수렴하는 레퍼런스 속도(main.cpp의 g_speed_ref)를 만들어 넘긴다 - PlanningLoop
    // 참고.
    double comfort_longitudinal_accel = 1.5;  // [m/s^2]
};

// BehaviorContext -> PlannerCommand. enable_*/forbid_* 방어적 검증 포함
// (문서 4.2절: "enable=false 후보는 만들거나 선택하지 않는 것이 안전함").
// obstacles/ref는 EMERGENCY의 물리 기반 stop_position이 장애물을 넘어서지
// 않도록 clamp하는 데만 쓰인다 (behavior_bridge.cpp 참고).
PlannerCommand BuildCommandFromContext(const behavior_planner::BehaviorContext& ctx,
                                        const FrenetState& ego_start,
                                        const BehaviorBridgeConfig& cfg,
                                        const std::vector<ObjectInfo>& obstacles,
                                        const RefLine& ref);

// 이번 사이클 결과 -> PlanFeedback. behavior_planner가 지금 이 값을
// BehaviorDecision에 반영하지 않는다고 문서에 명시돼 있어(4.3절 "통합
// 주의점"), 필드 정밀도보다 스키마를 끊기지 않게 채우는 것을 우선함.
behavior_planner::PlanFeedback BuildFeedback(const PlannerCommand& cmd_used,
                                              const FrenetPath* best,
                                              const behavior_planner::BehaviorContext& ctx);

#endif
