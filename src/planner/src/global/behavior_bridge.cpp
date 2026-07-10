#include "global/behavior_bridge.hpp"

#include <ros/ros.h>

namespace {

// stop_before_s가 신호/정지선 기반으로 세팅되지 않으면 HardConstraint 기본값
// 1e9(sentinel)로 남는다 (behavior_planner/src/context/behavior_context.hpp:7).
constexpr double kStopBeforeSSentinel = 1e8;

// 자차 전방(같은 차선)에서 가장 가까운 장애물의 s를 찾는다. EMERGENCY의
// 물리 기반 stop_position이 실제 장애물을 넘어서 계산되는 걸 막기 위한
// 용도로만 쓰인다 (behavior_bridge.hpp 설계 노트 참고). 못 찾으면 1e9.
double FindNearestObstacleSAhead(const FrenetState& ego_start,
                                  const std::vector<ObjectInfo>& obstacles,
                                  const RefLine& ref,
                                  double lane_width) {
    double nearest_s = 1e9;
    for (const auto& obj : obstacles) {
        CartesianState ocs{};
        ocs.x = obj.x;
        ocs.y = obj.y;
        ocs.yaw = obj.heading;

        double s, s_dot, s_ddot, d, d_prime, d_pprime;
        CartesianToFrenet(ref, ocs, s, s_dot, s_ddot, d, d_prime, d_pprime);

        const bool ahead = s > ego_start.s;
        const bool same_lane = std::abs(d - ego_start.d) < lane_width * 0.5;
        if (ahead && same_lane) nearest_s = std::min(nearest_s, s);
    }
    return nearest_s;
}

}  // namespace

PlannerCommand BuildCommandFromContext(const behavior_planner::BehaviorContext& ctx,
                                        const FrenetState& ego_start,
                                        const BehaviorBridgeConfig& cfg,
                                        const std::vector<ObjectInfo>& obstacles,
                                        const RefLine& ref) {
    PlannerCommand cmd{};
    cmd.target_speed = ctx.desired_speed;
    if (ctx.max_speed > 0.0) cmd.target_speed = std::min(cmd.target_speed, ctx.max_speed);

    BehaviorState mode = LANE_KEEPING;

    // force_stop/emergency_stop은 selected_behavior와 무관하게 우선 (문서 4.2절:
    // "force_stop=true 또는 selected_behavior=STOP 이면..."). 점수표상 항상
    // 일치해야 정상이지만, stale message 대비 방어적으로 이중 체크.
    if (ctx.emergency_stop || ctx.selected_behavior == behavior_planner::BehaviorContext::EMERGENCY_STOP) {
        if (ctx.enable_emergency_stop) {
            mode = EMERGENCY;
        } else {
            ROS_WARN_THROTTLE(1.0, "[BehaviorBridge] emergency_stop=true but enable_emergency_stop=false, "
                                    "falling back to LANE_KEEPING");
        }
    } else if (ctx.force_stop || ctx.selected_behavior == behavior_planner::BehaviorContext::STOP) {
        if (ctx.enable_stop) {
            mode = STOP;
        } else {
            ROS_WARN_THROTTLE(1.0, "[BehaviorBridge] force_stop=true but enable_stop=false, "
                                    "falling back to LANE_KEEPING");
        }
    } else {
        switch (ctx.selected_behavior) {
            case behavior_planner::BehaviorContext::FOLLOW:
                // 문서 4.3절: "속도 프로파일을 desired_speed 이하로 제한하고 앞차
                // 후방 비용 강화" — 후반부(비용 강화)는 cost.hpp에 대응 항이 없어
                // 이번 스코프에서는 속도 캡 + 기존 FilterByCollision으로 근사
                // (INTEGRATION_PLAN.md 1.1.1). 그래서 종방향 생성기는 그냥
                // VelocityKeeping(LANE_KEEPING)을 재사용한다.
                if (ctx.enable_follow) {
                    mode = LANE_KEEPING;
                } else {
                    ROS_WARN_THROTTLE(1.0, "[BehaviorBridge] selected_behavior=FOLLOW but "
                                            "enable_follow=false, falling back to LANE_KEEPING");
                }
                break;

            case behavior_planner::BehaviorContext::LEFT_CHANGE:
                if (ctx.enable_left_change && !ctx.forbid_left_change && !ctx.forbid_lane_change) {
                    mode = LANE_CHANGE_LEFT;
                    cmd.target_lane = -1;
                } else {
                    ROS_WARN_THROTTLE(1.0, "[BehaviorBridge] selected_behavior=LEFT_CHANGE but "
                                            "enable/forbid flags disagree, falling back to LANE_KEEPING");
                }
                break;

            case behavior_planner::BehaviorContext::RIGHT_CHANGE:
                if (ctx.enable_right_change && !ctx.forbid_right_change && !ctx.forbid_lane_change) {
                    mode = LANE_CHANGE_RIGHT;
                    cmd.target_lane = 1;
                } else {
                    ROS_WARN_THROTTLE(1.0, "[BehaviorBridge] selected_behavior=RIGHT_CHANGE but "
                                            "enable/forbid flags disagree, falling back to LANE_KEEPING");
                }
                break;

            case behavior_planner::BehaviorContext::YIELD:
                // behavior_decision.cpp의 BehaviorScore에 yield 필드 자체가 없어
                // 절대 발행되지 않는 값이지만, 문서(4.2절 해석표에 값이 없음)가
                // 명시한 대로 STOP과 동일하게 방어적으로 처리.
                if (ctx.enable_stop) {
                    mode = STOP;
                } else {
                    ROS_WARN_THROTTLE(1.0, "[BehaviorBridge] selected_behavior=YIELD but "
                                            "enable_stop=false, falling back to LANE_KEEPING");
                }
                break;

            case behavior_planner::BehaviorContext::KEEP:
                if (!ctx.enable_keep) {
                    ROS_WARN_THROTTLE(1.0, "[BehaviorBridge] selected_behavior=KEEP but "
                                            "enable_keep=false (unexpected)");
                }
                mode = LANE_KEEPING;
                break;

            default:
                mode = LANE_KEEPING;
                break;
        }
    }

    cmd.mode = mode;

    if (mode == STOP) {
        cmd.stop_position = ctx.stop_before_s;
    } else if (mode == EMERGENCY) {
        cmd.target_speed = 0.0;
        if (ctx.stop_before_s >= kStopBeforeSSentinel) {
            // FSM은 "EMERGENCY 상황"만 알려주고 물리적 정지거리는 안 준다 (TTC/
            // emergency_risk 트리거는 stop_before_s를 sentinel로 남겨둠 - hard_rule_filter.cpp
            // 확인됨). planner가 현재 속도와 차량의 감속 한계로 직접 계산한다.
            const double stop_distance = (ego_start.s_d * ego_start.s_d) /
                (2.0 * cfg.emergency_max_decel * cfg.emergency_decel_margin);
            double stop_s = ego_start.s + stop_distance + cfg.emergency_reaction_buffer;

            // 위 값이 실제 장애물을 넘어서면 모든 STOP 후보가 장애물을 관통해야
            // 해서 FilterByCollision이 전부 걸러내는 역설이 생긴다(실차 검증,
            // 2026-07-08). 장애물이 있으면 그 앞에서 멈추도록 clamp.
            const double obstacle_s = FindNearestObstacleSAhead(ego_start, obstacles, ref, cfg.lane_width);
            if (obstacle_s < 1e8) {
                stop_s = std::min(stop_s, obstacle_s - cfg.emergency_obstacle_margin);
            }
            cmd.stop_position = stop_s;
        } else {
            cmd.stop_position = ctx.stop_before_s;
        }
    }

    return cmd;
}

behavior_planner::PlanFeedback BuildFeedback(const PlannerCommand& cmd_used,
                                              const FrenetPath* best,
                                              const behavior_planner::BehaviorContext& ctx) {
    behavior_planner::PlanFeedback fb;
    fb.header.stamp = ros::Time::now();
    fb.plan_valid = (best != nullptr);

    switch (cmd_used.mode) {
        case LANE_CHANGE_LEFT:
            fb.selected_behavior = behavior_planner::BehaviorContext::LEFT_CHANGE;
            break;
        case LANE_CHANGE_RIGHT:
            fb.selected_behavior = behavior_planner::BehaviorContext::RIGHT_CHANGE;
            break;
        case STOP:
            fb.selected_behavior = behavior_planner::BehaviorContext::STOP;
            break;
        case EMERGENCY:
            fb.selected_behavior = behavior_planner::BehaviorContext::EMERGENCY_STOP;
            break;
        case LANE_KEEPING:
        default:
            fb.selected_behavior = behavior_planner::BehaviorContext::KEEP;
            break;
    }

    fb.selected_target_link_id = fb.plan_valid ? ctx.target_link_id : "";
    fb.previous_plan_still_valid = fb.plan_valid;

    // lane_width가 no-op이던 시절과 달리 이제 실제 오프셋을 쓰지만, 진행률을
    // 추적할 "차선변경 시작 시점" 상태가 planner에 없어 이번 스코프에서는
    // 단순화 (INTEGRATION_PLAN.md 2번). behavior_planner도 이 값을 아직
    // BehaviorDecision에 반영하지 않음(문서 "통합 주의점").
    fb.lane_change_completed = false;
    fb.lane_change_progress = 0.0;

    fb.selected_cost = best ? best->cost_total : 0.0;

    // FilterByCollision이 지금 pass/fail만 반환해서 실제 여유거리를 안 남김.
    // 의미 있는 값을 채우려면 collision_checker에 최소 여유거리 기록 확장이
    // 별도로 필요 (INTEGRATION_PLAN.md 2번) — 미측정 sentinel로 고정.
    fb.min_collision_margin = 1e9;

    return fb;
}
