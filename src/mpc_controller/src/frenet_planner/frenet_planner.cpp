#include "frenet_planner/frenet_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "global/utils.hpp"

namespace {

// 곡률 -> 목표속도 (mpc_controller 옛 planner/path_planner.cpp의
// velocityFromCurvature 이식).
double VelocityFromCurvature(double k, const CurveSpeedConfig& cfg) {
    if (k > cfg.curve_th_sharp) return cfg.curve_vel_sharp;
    if (k > cfg.curve_th_mid)   return cfg.curve_vel_mid;
    if (k > cfg.curve_th_mild)  return cfg.curve_vel_mild;
    return cfg.target_vel;
}

// 전방 lookahead 구간 내 최대 곡률로 미리 감속 (옛 lookaheadCurvature 이식).
//
// cfg.curve_lookahead_m(15m 고정)은 저속에서는 충분하지만, 고속(예: 9.5m/s로
// 진입)에서는 필요한 제동거리보다 짧아서 "곡선 진입 15m 전"에야 목표속도가
// curve_vel_sharp로 뚝 떨어지는데, 그 지점부터 급감속을 해도 다 못 줄이고
// 코너에 진입 -> 실제 곡률 대비 속도가 과도해 lateral 후보가 전부 무효화되는
// 문제가 실측으로 확인됐다(코너 진입 시 우회전 하려다 후보가 사라져 정지).
// 제동거리는 속도 제곱에 비례하므로(v0^2-v1^2)/(2a), 현재 속도 기준으로 그
// 거리를 계산해 15m와 비교해 더 큰 쪽을 쓴다. 실제 궤적은 jerk 비용 때문에
// max_longitudinal_accel 전부를 안 쓰므로(더 부드러운 감속을 선호), 그 절반
// 정도만 "편하게 낼 수 있는 감속도"로 보수적으로 잡고 여유 거리도 더한다.
double LookaheadTargetSpeed(const RefLine& ref, double s_start, double ego_speed,
                             const CurveSpeedConfig& cfg, double max_longitudinal_accel) {
    constexpr double kComfortableDecelFactor = 0.5;  // 실제 평균 감속도는 max의 절반 정도로 가정
    constexpr double kReactionMargin = 5.0;          // [m] 추가 여유거리
    const double usable_decel = std::max(0.5, max_longitudinal_accel * kComfortableDecelFactor);
    const double speed_drop = std::max(0.0, ego_speed * ego_speed - cfg.curve_vel_sharp * cfg.curve_vel_sharp);
    const double required_dist = speed_drop / (2.0 * usable_decel);
    const double lookahead_m = std::max(cfg.curve_lookahead_m, required_dist + kReactionMargin);

    double max_k = 0.0;
    for (const auto& p : ref.points) {
        if (p.s < s_start) continue;
        if (p.s > s_start + lookahead_m) break;
        max_k = std::max(max_k, std::fabs(p.kappa));
    }
    return VelocityFromCurvature(max_k, cfg);
}

}  // namespace

bool FrenetPlanner::Init(const std::string& yaml_path) {
    std::string waypoint_file;
    LoadParams(yaml_path, path_cfg_, limits_, cost_weights_, vehicle_shape_,
               collision_cfg_, curve_speed_cfg_, wheelbase_, lane_width_, waypoint_file);

    if (waypoint_file.empty() || !LoadReferenceLine(waypoint_file, ref_, limits_.max_curvature)) {
        std::printf("[FrenetPlanner] Reference line load failed (waypoint_file='%s')\n",
                    waypoint_file.c_str());
        return false;
    }
    ref_loaded_ = true;
    return true;
}

bool FrenetPlanner::Plan(const CartesianState& ego, const std::vector<ObjectInfo>& obstacles,
                          CartesianPath& out_path) {
    if (!ref_loaded_) return false;

    double s, s_dot, s_ddot, d, d_prime, d_pprime;
    CartesianToFrenet(ref_, ego, s, s_dot, s_ddot, d, d_prime, d_pprime,
                       has_last_s_ ? &last_s_ : nullptr);
    last_s_ = s;
    has_last_s_ = true;

    double d_dot, d_ddot;
    ArcDerivToTimeDeriv(s_dot, s_ddot, d_prime, d_pprime, d_dot, d_ddot);

    // CartesianToFrenet의 s_ddot/d_ddot 역산은 "고속 모드" 근사식이라, 코너
    // 진입 시(참조선은 이미 휘기 시작했는데 ego 실제 yaw는 아직 못 따라가서
    // delta_theta가 벌어지는 구간) s_dot^2 항이 이 오차를 그대로 증폭시켜
    // 실제로는 멀쩡한 상황인데도 현재 상태(start)의 s_ddot/d_ddot 자체가
    // max_longitudinal_accel/max_lateral_accel을 넘어버리는 경우가 있었다
    // (실측 재현: 코너 진입 직전 고속 구간에서 "No valid candidate (0 generated)"가
    // 수십 사이클 연속). 모든 후보가 이 t=0 경계값을 그대로 물려받으므로
    // start 자체가 한계를 넘으면 어떤 후보를 만들어도 전부 무효화된다. 실측
    // IMU 가속도(ego.a)가 이미 물리적으로 타당한 값이므로, 이 역산 결과를
    // 안전 한계 안으로 clamp해 "현재 상태 추정 오차"가 후보 생성 자체를
    // 막지 않게 한다.
    // 정확히 한계값(limit)으로 clamp하면, 그 값을 경계조건으로 받는 quintic/quartic이
    // 다항식 특성상 구간 중간에 경계값을 "오버슈트"할 수 있어(끝점 조건만 맞추고
    // 중간값은 별도 보장이 없음) 저속 구간(d1 후보가 start.d 하나뿐인 예외 처리)
    // 에서 그 유일한 후보마저 필터에 걸려 후보가 전혀 안 나오는 채로 영원히
    // 갇히는 경우가 실측으로 확인됐다(코너 끝 저속 구간에서 "0 generated"가
    // 회복 안 되고 계속됨 - 정지 상태가 유지되니 재시도해도 같은 결과만 반복).
    // 오버슈트 여유를 위해 한계값의 80%만 경계조건으로 허용한다.
    constexpr double kBoundaryMargin = 0.8;
    s_ddot = clip(s_ddot, -limits_.max_longitudinal_accel * kBoundaryMargin,
                  limits_.max_longitudinal_accel * kBoundaryMargin);
    d_ddot = clip(d_ddot, -limits_.max_lateral_accel * kBoundaryMargin,
                  limits_.max_lateral_accel * kBoundaryMargin);

    FrenetState start{s, s_dot, s_ddot, d, d_dot, d_ddot};

    // FSM 미완성: LANE_KEEPING 고정 + 곡률 기반 사전 감속만 반영.
    PlannerCommand cmd{};
    cmd.mode = LANE_KEEPING;
    cmd.target_speed = LookaheadTargetSpeed(ref_, s, ego.v, curve_speed_cfg_,
                                             limits_.max_longitudinal_accel);

    // 임시 브릿지 (FSM 완성 전): 정식 FSM이 아직 없어서 지금은 여기서 직접
    // "전방 lookahead 안에 내 차선과 겹치는 장애물이 있으면 AVOID로 전환"만
    // 판단한다. AVOID 모드/SAT 기반 FilterByCollision이 실제로 회피 경로를
    // 만들어내는지 검증하기 위한 최소 트리거이고, FSM이 완성되면 이 블록은
    // FSM이 넘겨주는 cmd로 교체될 예정이다.
    constexpr double kAvoidLookaheadM = 30.0;    // [m] 이 거리 안의 장애물만 회피 트리거로 봄
    constexpr double kAvoidLaneHalfWidth = 1.2;  // [m] 장애물 중심이 이 안이면 "내 차선"으로 판정
    for (const auto& obs : obstacles) {
        const double obs_s = FindClosestS(ref_, obs.x, obs.y, &s);
        if (obs_s < s || obs_s > s + kAvoidLookaheadM) continue;
        const RefPoint rp = Interpolate(ref_, obs_s);
        const double obs_d = (obs.x - rp.x) * (-std::sin(rp.theta)) + (obs.y - rp.y) * std::cos(rp.theta);
        if (std::fabs(obs_d) < kAvoidLaneHalfWidth) {
            cmd.mode = AVOID;
            cmd.avoidance_d_offset = (obs_d >= 0.0) ? -lane_width_ : lane_width_;
            std::printf("[FrenetPlanner] AVOID triggered: obstacle id=%d obs_s=%.2f obs_d=%.2f "
                        "-> avoidance_d_offset=%.2f\n",
                        obs.id, obs_s, obs_d, cmd.avoidance_d_offset);
            break;
        }
    }

    PlannerDebugStats stats;
    std::vector<FrenetPath> candidates =
        ResolveManeuver(start, cmd, ref_, path_cfg_, limits_, lane_width_, &stats);

    FilterByCollision(candidates, ref_, obstacles, vehicle_shape_, collision_cfg_);
    EvaluateCosts(candidates, cost_weights_);

    const FrenetPath* best = SelectBestPath(candidates);
    if (!best) {
        size_t collision_valid = 0;
        for (const auto& p : candidates) if (p.valid) collision_valid++;
        std::printf("[FrenetPlanner] No valid candidate this cycle (%zu generated)\n", candidates.size());
        std::printf("[FrenetPlanner-DEBUG] ego: x=%.3f y=%.3f yaw=%.3f v=%.3f a=%.3f\n",
                     ego.x, ego.y, ego.yaw, ego.v, ego.a);
        std::printf("[FrenetPlanner-DEBUG] start: s=%.2f d=%.3f s_dot=%.2f s_ddot=%.2f d_dot=%.3f d_ddot=%.3f "
                     "target_speed=%.2f\n",
                     s, d, s_dot, s_ddot, d_dot, d_ddot, cmd.target_speed);
        std::printf("[FrenetPlanner-DEBUG] lateral %zu/%zu valid | longitudinal %zu/%zu valid | "
                     "combined %zu -> after curvature %zu valid -> after collision %zu valid\n",
                     stats.lateral_valid, stats.lateral_total,
                     stats.longitudinal_valid, stats.longitudinal_total,
                     stats.combined_total, stats.combined_valid_after_curvature, collision_valid);
        return false;
    }

    out_path = ConvertToCartesianPath(*best, ref_);
    return true;
}
