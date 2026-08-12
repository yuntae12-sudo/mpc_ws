#include "frenet_planner/frenet/path_generator.hpp"
#include "frenet_planner/math/polynomial.hpp"
#include "frenet_planner/math/frenet_converter.hpp"

#include <algorithm>
#include <cmath>

// =========================================================
// SamplingRange(min,max,step) -> 실제 후보값 목록
// =========================================================
std::vector<double> SampleRange(const SamplingRange& r) {
    std::vector<double> vals;
    if (r.step <= 1e-9) {
        vals.push_back(r.min);
        return vals;
    }
    for (double v = r.min; v <= r.max + 1e-9; v += r.step) {
        vals.push_back(v);
    }
    return vals;
}

// 0부터 T까지 dt 간격의 시간 샘플 (끝점 T는 항상 정확히 포함)
std::vector<double> SampleTimes(double T, double dt) {
    std::vector<double> times;
    for (double t = 0.0; t < T - 1e-9; t += dt) {
        times.push_back(t);
    }
    times.push_back(T);
    return times;
}

// 횡방향 quintic 하나를 시간 샘플에 맞춰 FrenetPath로 변환 (d, d_d, d_dd만 채움)
FrenetPath SampleLateralQuintic(const QuinticPolynomial& poly, const std::vector<double>& times) {
    FrenetPath path;
    path.t = times;
    path.valid = true;
    path.cost_lat = path.cost_lon = path.cost_total = 0.0;
    path.jerk_cost_lat = JerkCost(poly, times.back());
    path.jerk_cost_lon = 0.0;
    path.delta_s = 0.0;
    path.delta_s_dot = 0.0;
    path.d.reserve(times.size());
    path.d_d.reserve(times.size());
    path.d_dd.reserve(times.size());
    for (double t : times) {
        path.d.push_back(EvalPos(poly, t));
        path.d_d.push_back(EvalVel(poly, t));
        path.d_dd.push_back(EvalAcc(poly, t));
    }
    return path;
}

// 종방향 quintic 하나를 시간 샘플에 맞춰 FrenetPath로 변환 (s, s_d, s_dd만 채움)
FrenetPath SampleLongitudinalQuintic(const QuinticPolynomial& poly, const std::vector<double>& times) {
    FrenetPath path;
    path.t = times;
    path.valid = true;
    path.cost_lat = path.cost_lon = path.cost_total = 0.0;
    path.jerk_cost_lat = 0.0;
    path.jerk_cost_lon = JerkCost(poly, times.back());
    path.delta_s = 0.0;
    path.delta_s_dot = 0.0;
    path.s.reserve(times.size());
    path.s_d.reserve(times.size());
    path.s_dd.reserve(times.size());
    for (double t : times) {
        path.s.push_back(EvalPos(poly, t));
        path.s_d.push_back(EvalVel(poly, t));
        path.s_dd.push_back(EvalAcc(poly, t));
    }
    return path;
}

// 종방향 quartic 하나(velocity keeping)를 시간 샘플에 맞춰 FrenetPath로 변환
FrenetPath SampleLongitudinalQuartic(const QuarticPolynomial& poly, const std::vector<double>& times) {
    FrenetPath path;
    path.t = times;
    path.valid = true;
    path.cost_lat = path.cost_lon = path.cost_total = 0.0;
    path.jerk_cost_lat = 0.0;
    path.jerk_cost_lon = JerkCost(poly, times.back());
    path.delta_s = 0.0;
    path.delta_s_dot = 0.0;
    path.s.reserve(times.size());
    path.s_d.reserve(times.size());
    path.s_dd.reserve(times.size());
    for (double t : times) {
        path.s.push_back(EvalPos(poly, t));
        path.s_d.push_back(EvalVel(poly, t));
        path.s_dd.push_back(EvalAcc(poly, t));
    }
    return path;
}

// =========================================================
// [Sec.IV-A] 횡방향 후보 집합 (고속 모드, d(t) quintic)
// =========================================================

std::vector<FrenetPath> GenerateLateralCandidates(const FrenetState& start,
                                                   const PathGeneratorConfig& cfg) {
    std::vector<FrenetPath> result;

    // 저속 임시 처리 (path_generator.hpp 주석 참고): 이 속도 미만에서는
    // d(t) quintic이 물리적으로 불가능한(옆으로 미끄러지는) 후보만 만들어
    // 곡률 필터에서 전부 걸러지므로, d1 격자 대신 현재 d를 유지하는 후보
    // 하나만 생성해 최소한 직진 가속은 가능하게 한다.
    // 저속 단일 후보(start.d)만 두면, 그 하나가 어떤 이유로든 필터에 걸리는
    // 순간(예: 경계조건 오버슈트) 대안이 아예 없어 "정지 -> 후보 0개 -> 정지
    // 유지"로 영원히 갇히는 문제가 실측으로 확인됐다(속도가 임계값을 못 넘어
    // 저속 분기를 못 벗어남). start.d와 다르면(=0 근처가 아니면) 차선 중앙
    // (d1=0) 후보도 같이 만들어 최소한 하나는 살아남을 여지를 준다.
    constexpr double kLowSpeedThreshold = 0.5;  // [m/s]
    std::vector<double> d1_list;
    if (std::fabs(start.s_d) < kLowSpeedThreshold) {
        d1_list.push_back(start.d);
        if (std::fabs(start.d) > 1e-3) d1_list.push_back(0.0);
    } else {
        d1_list = SampleRange(cfg.lateral_d1);
    }
    std::vector<double> T_list  = SampleRange(cfg.time_horizon);

    for (double T : T_list) {
        if (T <= 1e-6) continue;  // MakeQuintic의 T 분모 보호
        std::vector<double> times = SampleTimes(T, cfg.dt);

        for (double d1 : d1_list) {
            // 종료조건 [d1, 0, 0, T] — 논문 Sec.IV-A, "차선과 평행하게(dot=ddot=0) 복귀"
            QuinticPolynomial poly = MakeQuintic(start.d, start.d_d, start.d_dd,
                                                  d1, 0.0, 0.0, T);
            result.push_back(SampleLateralQuintic(poly, times));
        }
    }

    return result;
}

// =========================================================
// [Sec.V-A] Following
// =========================================================

std::vector<FrenetPath> GenerateFollowingCandidates(const FrenetState& start,
                                                     double leader_s,
                                                     double leader_speed,
                                                     double leader_accel,
                                                     double time_gap,
                                                     double min_gap,
                                                     double cruise_target_speed,
                                                     double gap_gain,
                                                     const PathGeneratorConfig& cfg) {
    std::vector<FrenetPath> result;

    std::vector<double> delta_list = SampleRange(cfg.delta_s);
    std::vector<double> T_list     = SampleRange(cfg.time_horizon);

    // Constant-time-gap 오차를 접근 속도로 피드백한다. 목표 간격보다 멀면
    // leader보다 빠르게 접근하고, 목표 간격에서는 leader 속도와 같아진다.
    // 상한은 도로 곡률 등을 반영해 상위 planner가 정한 순항 목표속도이다.
    const double current_gap = leader_s - start.s;
    const double desired_gap_now = min_gap + time_gap * leader_speed;
    const double gap_error = current_gap - desired_gap_now;
    const double raw_approach_speed = leader_speed + gap_gain * gap_error;
    const double approach_speed = std::max(0.0, std::min(cruise_target_speed,
                                                         raw_approach_speed));

    for (double T : T_list) {
        if (T <= 1e-6) continue;
        std::vector<double> times = SampleTimes(T, cfg.dt);

        // 선두 차량 등가속(leader_accel = const) 예측을 T 시점까지 시간적분
        const double s_lv_T     = leader_s + leader_speed * T + 0.5 * leader_accel * T * T;
        const double s_lv_dot_T = leader_speed + leader_accel * T;

        // s_follow(t) = s_lv(t) - [D0 + tau*s_lv_dot(t)] (D0=min_gap, tau=time_gap).
        // leader가 멀리 있을 때 following 지점을 한 horizon 안에 강제로 맞추면
        // 비현실적인 quintic이 된다. 그 경우에는 현재 속도를 고정하지 않고,
        // 위의 간격 피드백으로 계산한 접근 속도까지 부드럽게 천이한다.
        const double s_follow_T = s_lv_T - (min_gap + time_gap * s_lv_dot_T);
        const double free_s_T = start.s + 0.5 * (start.s_d + approach_speed) * T;
        // s_dot_target(t) = s_lv_dot(t) - tau*s_lv_ddot(t), s_lv_ddot=leader_accel(상수)
        const double s_follow_dot_T = s_lv_dot_T - time_gap * leader_accel;
        // s_ddot_target(t) = s_lv_ddot(t1) = leader_accel (상수, jerk=0 가정)
        const double s_follow_ddot_T = leader_accel;

        const bool gap_control_needed = s_follow_T <= free_s_T;
        const double s_target_T = gap_control_needed ? s_follow_T : free_s_T;
        const double s_target_dot_T = gap_control_needed ? s_follow_dot_T : approach_speed;
        const double s_target_ddot_T = gap_control_needed ? s_follow_ddot_T : 0.0;

        for (double delta : delta_list) {
            const double s1 = s_target_T + delta;
            QuinticPolynomial poly = MakeQuintic(start.s, start.s_d, start.s_dd,
                                                  s1, s_target_dot_T, s_target_ddot_T, T);
            FrenetPath path = SampleLongitudinalQuintic(poly, times);
            path.delta_s = delta;   // Ct의 [s1-sd]^2 = delta^2 항에 쓰임
            result.push_back(std::move(path));
        }
    }

    return result;
}

// =========================================================
// [Sec.V-A] Stopping
// =========================================================

std::vector<FrenetPath> GenerateStoppingCandidates(const FrenetState& start,
                                                    double stop_s,
                                                    const PathGeneratorConfig& cfg) {
    std::vector<FrenetPath> result;

    std::vector<double> delta_list = SampleRange(cfg.delta_s);
    std::vector<double> T_list     = SampleRange(cfg.time_horizon);

    for (double T : T_list) {
        if (T <= 1e-6) continue;
        std::vector<double> times = SampleTimes(T, cfg.dt);

        // s_target = s_stop (상수), s_dot_target = 0, s_ddot_target = 0  (논문 명시)
        for (double delta : delta_list) {
            const double s1 = stop_s + delta;
            QuinticPolynomial poly = MakeQuintic(start.s, start.s_d, start.s_dd,
                                                  s1, 0.0, 0.0, T);
            FrenetPath path = SampleLongitudinalQuintic(poly, times);
            path.delta_s = delta;
            result.push_back(std::move(path));
        }
    }

    return result;
}

// =========================================================
// [Sec.V-A] Merging
// =========================================================

std::vector<FrenetPath> GenerateMergingCandidates(const FrenetState& start,
                                                   double sa, double sa_speed, double sa_accel,
                                                   double sb, double sb_speed, double sb_accel,
                                                   const PathGeneratorConfig& cfg) {
    std::vector<FrenetPath> result;

    std::vector<double> delta_list = SampleRange(cfg.delta_s);
    std::vector<double> T_list     = SampleRange(cfg.time_horizon);

    for (double T : T_list) {
        if (T <= 1e-6) continue;
        std::vector<double> times = SampleTimes(T, cfg.dt);

        // sa(t), sb(t) 각각 등가속 예측 (Following과 동일한 적분, time-gap 항은 없음)
        const double sa_T     = sa + sa_speed * T + 0.5 * sa_accel * T * T;
        const double sa_dot_T = sa_speed + sa_accel * T;
        const double sb_T     = sb + sb_speed * T + 0.5 * sb_accel * T * T;
        const double sb_dot_T = sb_speed + sb_accel * T;

        // s_target(t) = 1/2 [sa(t)+sb(t)]  (논문 식 (3))
        const double s_target_T      = 0.5 * (sa_T + sb_T);
        const double s_target_dot_T  = 0.5 * (sa_dot_T + sb_dot_T);
        const double s_target_ddot_T = 0.5 * (sa_accel + sb_accel);

        for (double delta : delta_list) {
            const double s1 = s_target_T + delta;
            QuinticPolynomial poly = MakeQuintic(start.s, start.s_d, start.s_dd,
                                                  s1, s_target_dot_T, s_target_ddot_T, T);
            FrenetPath path = SampleLongitudinalQuintic(poly, times);
            path.delta_s = delta;
            result.push_back(std::move(path));
        }
    }

    return result;
}

std::vector<FrenetPath> GenerateMergeArrivalCandidates(const FrenetState& start,
                                                        double conflict_s,
                                                        double entry_time,
                                                        double entry_speed,
                                                        const PathGeneratorConfig& cfg) {
    std::vector<FrenetPath> result;
    // lateral 후보와 정확히 같은 T 격자를 써야 Combine 단계에서 결합된다.
    // 선택 entry_time 주변의 격자만 남겨 시간 목표를 유지한다.
    for (double T = cfg.time_horizon.min;
         T <= cfg.time_horizon.max + 1e-9; T += cfg.time_horizon.step) {
        if (T <= 1e-6) continue;
        if (std::fabs(T - entry_time) > std::max(0.6, cfg.time_horizon.step)) continue;
        const std::vector<double> times = SampleTimes(T, cfg.dt);
        // 위치 종단조건을 느슨한 평균속도로 대체하지 않고 conflict_s에 직접 고정한다.
        QuinticPolynomial poly = MakeQuintic(start.s, start.s_d, start.s_dd,
                                              conflict_s, entry_speed, 0.0, T);
        result.push_back(SampleLongitudinalQuintic(poly, times));
    }
    // entry_time이 격자 밖인 예외에는 가장 가까운 horizon 하나를 사용한다.
    if (result.empty()) {
        const double grid_index = std::round(
            (entry_time - cfg.time_horizon.min) / cfg.time_horizon.step);
        const double T = std::max(cfg.time_horizon.min,
            std::min(cfg.time_horizon.min + grid_index * cfg.time_horizon.step,
                     cfg.time_horizon.max));
        const std::vector<double> times = SampleTimes(T, cfg.dt);
        QuinticPolynomial poly = MakeQuintic(start.s, start.s_d, start.s_dd,
                                              conflict_s, entry_speed, 0.0, T);
        result.push_back(SampleLongitudinalQuintic(poly, times));
    }
    return result;
}

// =========================================================
// [Sec.V-B] Velocity Keeping
// =========================================================

std::vector<FrenetPath> GenerateVelocityKeepingCandidates(const FrenetState& start,
                                                           double target_speed,
                                                           const PathGeneratorConfig& cfg) {
    std::vector<FrenetPath> result;

    std::vector<double> dsdot_list = SampleRange(cfg.delta_s_dot);
    std::vector<double> T_list     = SampleRange(cfg.time_horizon);

    for (double T : T_list) {
        if (T <= 1e-6) continue;
        std::vector<double> times = SampleTimes(T, cfg.dt);

        for (double dsdot : dsdot_list) {
            // 종료조건 [s_dot_target, 0, T] — 위치 자유, 종료 가속도 항상 0 (논문 명시)
            const double v1 = target_speed + dsdot;
            QuarticPolynomial poly = MakeQuartic(start.s, start.s_d, start.s_dd,
                                                  v1, 0.0, T);
            FrenetPath path = SampleLongitudinalQuartic(poly, times);
            path.delta_s_dot = dsdot;   // Cv의 [s_dot1-s_dot_d]^2 = dsdot^2 항에 쓰임
            result.push_back(std::move(path));
        }
    }

    return result;
}

// =========================================================
// [Sec.VI 1단] 결합 전 가속도 필터링
// =========================================================

void FilterLateralByAcceleration(std::vector<FrenetPath>& candidates,
                                  const KinematicLimits& limits) {
    for (auto& path : candidates) {
        if (!path.valid) continue;
        for (double dd : path.d_dd) {
            if (std::abs(dd) > limits.max_lateral_accel) {
                path.valid = false;
                break;
            }
        }
    }
}

void FilterLongitudinalByAcceleration(std::vector<FrenetPath>& candidates,
                                       const KinematicLimits& limits) {
    for (auto& path : candidates) {
        if (!path.valid) continue;
        for (double sdd : path.s_dd) {
            if (std::abs(sdd) > limits.max_longitudinal_accel) {
                path.valid = false;
                break;
            }
        }
    }
}

// =========================================================
// [Sec.VI 2단] Tlat x Tlon 결합 — 같은 T(종료시간)를 가진 후보끼리만 결합
// =========================================================

std::vector<FrenetPath> CombineLateralLongitudinal(const std::vector<FrenetPath>& lateral_set,
                                                    const std::vector<FrenetPath>& longitudinal_set) {
    std::vector<FrenetPath> result;

    for (const auto& lat : lateral_set) {
        if (!lat.valid || lat.t.empty()) continue;
        const double T_lat = lat.t.back();

        for (const auto& lon : longitudinal_set) {
            if (!lon.valid || lon.t.empty()) continue;
            const double T_lon = lon.t.back();

            if (std::abs(T_lat - T_lon) > 1e-6) continue;   // 같은 T만 결합
            if (lat.t.size() != lon.t.size()) continue;      // 안전장치 (동일 T, dt면 항상 같아야 함)

            FrenetPath combined;
            combined.t     = lat.t;
            combined.d     = lat.d;
            combined.d_d   = lat.d_d;
            combined.d_dd  = lat.d_dd;
            combined.s     = lon.s;
            combined.s_d   = lon.s_d;
            combined.s_dd  = lon.s_dd;
            combined.valid = true;
            combined.rejection_reason = RejectionReason::NONE;
            combined.cost_lat = combined.cost_lon = combined.cost_total = 0.0;
            combined.jerk_cost_lat = lat.jerk_cost_lat;
            combined.jerk_cost_lon = lon.jerk_cost_lon;
            combined.delta_s       = lon.delta_s;
            combined.delta_s_dot   = lon.delta_s_dot;

            result.push_back(std::move(combined));
        }
    }

    return result;
}

// =========================================================
// [Sec.VI 3단] 결합 이후 곡률 필터링
// =========================================================

void FilterByCurvature(std::vector<FrenetPath>& combined,
                        const RefLine& ref,
                        const KinematicLimits& limits) {
    // 시간미분->호길이미분(TimeDerivToArcDeriv, s_dot로 나눔) 기반 계산은 저속에서
    // 발산해 STOP/EMERGENCY처럼 정지에 수렴하는 후보를 전부 무효화시켰다
    // (frenet_converter.hpp의 ComputeGeometricPath 설계 노트 참고). 위치(x,y)
    // 기반 기하학적 곡률로 대체해 고속/저속 구분 없이 일관되게 판정한다.
    for (auto& path : combined) {
        if (!path.valid) continue;

        GeometricPath geo = ComputeGeometricPath(path.s, path.d, ref);
        for (double k : geo.kappa) {
            if (std::abs(k) > limits.max_curvature) {
                path.valid = false;
                path.rejection_reason = RejectionReason::CURVATURE;
                break;
            }
        }
    }
}

// =========================================================
// [Sec.VII] resolveManeuver
// =========================================================

// AVOID 모드는 PlannerCommand.avoidance_d_offset을 그대로 lateral 목표 중심으로 사용.
// LANE_CHANGE_*는 d 양의 방향이 좌측(FrenetToCartesian의 n_r 정의와 동일)이므로
// LEFT는 +lane_width, RIGHT는 -lane_width. 회전교차로 MERGE는 global path 유지.
double ResolveLateralOffset(const PlannerCommand& cmd, double lane_width) {
    switch (cmd.mode) {
        case AVOID:
            return cmd.avoidance_d_offset;

        case LANE_CHANGE_LEFT:
            return lane_width;

        case LANE_CHANGE_RIGHT:
            return -lane_width;

        case MERGE:
            return 0.0;  // 회전교차로 진입 global path 자체를 횡방향으로 추종

        default:
            return 0.0;  // 차선 중앙 유지
    }
}

std::vector<FrenetPath> ResolveManeuver(const FrenetState& start,
                                         const PlannerCommand& cmd,
                                         const RefLine& ref,
                                         const PathGeneratorConfig& cfg,
                                         const KinematicLimits& limits,
                                         double lane_width,
                                         const std::vector<ObjectInfo>& obstacles,
                                         const VehicleShape& vehicle_shape,
                                         const CollisionCheckConfig& collision_cfg,
                                         PlannerDebugStats* stats) {
    // 1. lateral 후보. 전 모드에 공유된 [-3,+3]m 격자를 쓰면
    // LANE_KEEPING에서도 먼 장애물을 보고 옆 후보가 선택된다. 모드가
    // 지정한 목표 d 주변의 작은 범위만 생성해 역할을 분리한다.
    PathGeneratorConfig lateral_cfg = cfg;
    // COMMIT/CROSS에서는 짧은 종방향 거리 안에 d=0 복귀까지 강제하지 않는다.
    // 현재 횡위치를 유지해 합류 타이밍 궤적과 횡복귀 궤적을 분리하고, CLEAR 후
    // LANE_KEEPING이 차선 중앙 복귀를 담당한다.
    const double d_offset = (cmd.mode == MERGE && cmd.merge_committed)
        ? start.d : ResolveLateralOffset(cmd, lane_width);
    // MERGE는 차선 변경 모드가 아니라 회전교차로 global path의 종방향 진입
    // 타이밍만 조절한다. ±0.5m 후보가 충돌 회피처럼 번갈아 선택되며 조향이
    // 흔들리지 않도록 종단 d를 차선 중심 하나로 고정한다.
    const double lateral_tolerance =
        cmd.mode == MERGE ? 0.0 : cfg.lateral_target_tolerance;
    lateral_cfg.lateral_d1.min = d_offset - lateral_tolerance;
    lateral_cfg.lateral_d1.max = d_offset + lateral_tolerance;

    std::vector<FrenetPath> lateral_set = GenerateLateralCandidates(start, lateral_cfg);
    FilterLateralByAcceleration(lateral_set, limits);
    if (stats) {
        stats->lateral_total = lateral_set.size();
        for (const auto& p : lateral_set) if (p.valid) stats->lateral_valid++;
    }

    // 2. longitudinal 후보 (FSM이 지정한 모드에 따라 생성 방식 선택)
    std::vector<FrenetPath> longitudinal_set;

    switch (cmd.mode) {
        case FOLLOWING:
            longitudinal_set = GenerateFollowingCandidates(start, cmd.leader_s, cmd.leader_speed,
                                                            cmd.leader_accel, cmd.time_gap,
                                                            cmd.min_gap, cmd.target_speed,
                                                            cmd.gap_gain, cfg);
            break;

        case STOP:
        case INTERSECTION_WAIT:
            longitudinal_set = GenerateStoppingCandidates(start, cmd.stop_position, cfg);
            break;

        case EMERGENCY:
            // FSM이 emergency 상황에 맞는 stop_position(예: 현재 위치 바로 앞)을
            // 설정해준다고 가정. path_generator는 물리적으로 안전한 제동거리를
            // 스스로 계산하지 않는다 (TODO: 추후 FSM과 함께 정책 확정 필요).
            longitudinal_set = GenerateStoppingCandidates(start, cmd.stop_position, cfg);
            break;

        case MERGE:
            if (cmd.merge_crossing) {
                if (cmd.leader_id >= 0) {
                    longitudinal_set = GenerateFollowingCandidates(
                        start, cmd.leader_s, cmd.leader_speed, cmd.leader_accel,
                        cmd.time_gap, cmd.min_gap, cmd.target_speed, cmd.gap_gain, cfg);
                } else {
                    longitudinal_set = GenerateVelocityKeepingCandidates(
                        start, cmd.target_speed, cfg);
                }
            } else if (!cmd.merge_gap_safe) {
                const double stop_distance = std::max(0.0, cmd.merge_stop_s - start.s);
                const double braking_distance = start.s_d * start.s_d /
                    (2.0 * std::max(limits.max_longitudinal_accel, 0.1));
                if (stop_distance > braking_distance + 3.0) {
                    // 먼 거리부터 정지 quintic을 강제하면 모든 T 후보가 조기 감속한다.
                    // 제동구간 전까지는 global-path 순항을 유지한다.
                    longitudinal_set = GenerateVelocityKeepingCandidates(
                        start, cmd.target_speed, cfg);
                } else {
                    longitudinal_set = GenerateStoppingCandidates(start, cmd.merge_stop_s, cfg);
                }
            } else {
                const double distance = std::max(0.0, cmd.merge_conflict_s - start.s);
                const double arrival_speed = cmd.merge_entry_time > 0.1
                    ? distance / cmd.merge_entry_time : cmd.target_speed;
                if (cmd.merge_committed &&
                    cmd.merge_entry_time >= cfg.time_horizon.min - 1e-6) {
                    longitudinal_set = GenerateMergeArrivalCandidates(
                        start, cmd.merge_conflict_s, cmd.merge_entry_time,
                        std::min(cmd.target_speed, std::max(arrival_speed, 1.0)), cfg);
                } else if (cmd.merge_committed) {
                    // conflict 도착까지 남은 시간이 lateral/longitudinal 공통
                    // 최소 horizon보다 짧아지면 exact-arrival quintic을 만들 수 없다.
                    // 기존 구현은 entry_time(예: 1.2s)을 1.5s로 강제 확장해 가까운
                    // conflict point에 늦게 도착하도록 급감속/재가속했고, 결국
                    // 종가속도 필터에서 유일한 후보가 사라져 zone 앞에서 정지했다.
                    // COMMIT은 이미 전체 후보 충돌 검증을 통과했으므로 이 구간부터는
                    // 교차구역 통과 목표속도를 유지해 시간창을 끝까지 실행한다.
                    longitudinal_set = GenerateVelocityKeepingCandidates(
                        start, cmd.target_speed, cfg);
                } else {
                    if (cmd.leader_id >= 0) {
                        longitudinal_set = GenerateFollowingCandidates(
                            start, cmd.leader_s, cmd.leader_speed, cmd.leader_accel,
                            cmd.time_gap, cmd.min_gap,
                            std::min(cmd.target_speed, std::max(arrival_speed, 1.0)),
                            cmd.gap_gain, cfg);
                    } else {
                        longitudinal_set = GenerateVelocityKeepingCandidates(
                            start, std::min(cmd.target_speed, std::max(arrival_speed, 1.0)), cfg);
                    }
                }
            }
            break;

        case LANE_KEEPING:
        case LANE_CHANGE_LEFT:
        case LANE_CHANGE_RIGHT:
        case TURN_LEFT:
        case TURN_RIGHT:
        case AVOID:
        default:
            // 논문 Sec.VII: velocity keeping은 앞 차가 없는 대부분의 상황(차선유지,
            // 차선변경, 회피 등)의 기본 종방향 모드.
            longitudinal_set = GenerateVelocityKeepingCandidates(start, cmd.target_speed, cfg);
            break;
    }

    FilterLongitudinalByAcceleration(longitudinal_set, limits);
    if (stats) {
        stats->longitudinal_total = longitudinal_set.size();
        for (const auto& p : longitudinal_set) if (p.valid) stats->longitudinal_valid++;
    }

    // 3. 결합 + 사후 곡률 필터링
    std::vector<FrenetPath> combined = CombineLateralLongitudinal(lateral_set, longitudinal_set);
    if (stats) stats->combined_total = combined.size();
    FilterByCurvature(combined, ref, limits);
    if (stats) {
        for (const auto& p : combined) if (p.valid) stats->combined_valid_after_curvature++;
    }

    // 4. 충돌 필터링(Sec.VI) - obstacles가 없으면 아무 것도 안 걸러지므로
    // 안전하게 항상 호출해도 된다(장애물 없을 때 기존 동작과 동일).
    // TODO(검증 필요): FilterByCollision은 exempt_id 하나만 받는데, MERGE는
    // sa/sb 둘 다와 가까운 게 정상이라 둘 다 면제가 필요할 수 있음. sa/sb
    // 간격이 충분히 넓다면 문제없겠지만, 좁은 gap으로 MERGE 검증 시 후보가
    // 0개로 걸러지면 이 지점부터 확인.
    if (!obstacles.empty()) {
        const int coast_exempt_id =
            (cmd.mode == FOLLOWING || cmd.mode == MERGE) ? cmd.leader_id : -1;
        FilterByCollision(combined, ref, obstacles, vehicle_shape, collision_cfg,
                          coast_exempt_id);
    }
    if (stats) {
        for (const auto& p : combined) if (p.valid) stats->combined_valid_after_collision++;
    }

    return combined;
}
