#include "frenet/path_generator.hpp"
#include "math/polynomial.hpp"
#include "math/frenet_converter.hpp"

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
// lateral/longitudinal이 같은 T, 같은 dt로 호출되면 항상 동일한 배열이 나와야
// CombineLateralLongitudinal에서 인덱스 대 인덱스로 그대로 합칠 수 있다.
std::vector<double> SampleTimes(double T, double dt) {
    std::vector<double> times;
    for (double t = 0.0; t < T - 1e-9; t += dt) {
        times.push_back(t);
    }
    times.push_back(T);
    return times;
}

// 횡방향 quintic 하나를 시간 샘플에 맞춰 FrenetPath로 변환 (d, d_d, d_dd만 채움)
// jerk_cost_lat = Jt(d(t))을 다항식 계수로부터 닫힌 형태로 미리 계산해둔다
// (계수는 이 함수를 벗어나면 사라지므로 cost.hpp가 나중에 계산할 방법이 없음).
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
// delta_s(Δsi)는 호출 측(Following/Stopping/Merging)이 채워준다.
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
// delta_s_dot(Δṡi)는 호출 측(GenerateVelocityKeepingCandidates)이 채워준다.
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

    std::vector<double> d1_list = SampleRange(cfg.lateral_d1);
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
                                                     const PathGeneratorConfig& cfg) {
    std::vector<FrenetPath> result;

    std::vector<double> delta_list = SampleRange(cfg.delta_s);
    std::vector<double> T_list     = SampleRange(cfg.time_horizon);

    for (double T : T_list) {
        if (T <= 1e-6) continue;
        std::vector<double> times = SampleTimes(T, cfg.dt);

        // 선두 차량 등가속(leader_accel = const) 예측을 T 시점까지 시간적분
        const double s_lv_T     = leader_s + leader_speed * T + 0.5 * leader_accel * T * T;
        const double s_lv_dot_T = leader_speed + leader_accel * T;

        // s_target(t) = s_lv(t) - [D0 + tau*s_lv_dot(t)]  (D0=min_gap, tau=time_gap)
        const double s_target_T     = s_lv_T - (min_gap + time_gap * s_lv_dot_T);
        // s_dot_target(t) = s_lv_dot(t) - tau*s_lv_ddot(t), s_lv_ddot=leader_accel(상수)
        const double s_target_dot_T = s_lv_dot_T - time_gap * leader_accel;
        // s_ddot_target(t) = s_lv_ddot(t1) = leader_accel (상수, jerk=0 가정)
        const double s_target_ddot_T = leader_accel;

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
//
// combined는 lateral을 고속(d(t)) 모드로 생성했으므로 d,d_d,d_dd는 시간미분.
// FrenetToCartesian은 arc-length 미분(d,d',d'')을 받으므로, 매 샘플마다
// TimeDerivToArcDeriv로 먼저 변환한다 (frenet_converter.hpp의 "고속 어댑터").
// s_dot이 0에 가까우면 이 어댑터 자체가 정의역을 벗어나므로(0 나눗셈),
// 해당 궤적은 곡률 계산 없이 바로 무효 처리한다.
// =========================================================

void FilterByCurvature(std::vector<FrenetPath>& combined,
                        const RefLine& ref,
                        const KinematicLimits& limits) {
    constexpr double kMinSpeedForCurvatureCheck = 0.1;  // [m/s], 고속 모드 전제 하한

    for (auto& path : combined) {
        if (!path.valid) continue;

        for (size_t i = 0; i < path.t.size(); i++) {
            if (std::abs(path.s_d[i]) < kMinSpeedForCurvatureCheck) {
                // TODO(추후 개발 필요, FSM 저속 처리와 함께): 저속에서는 d(s) 모드 +
                // 별도 곡률 계산이 필요. 지금은 고속 모드 전제가 깨지므로 무효 처리.
                path.valid = false;
                break;
            }

            double d_prime, d_pprime;
            TimeDerivToArcDeriv(path.s_d[i], path.s_dd[i], path.d_d[i], path.d_dd[i],
                                 d_prime, d_pprime);

            RefPoint rp = Interpolate(ref, path.s[i]);
            CartesianState cs = FrenetToCartesian(rp, path.s[i], path.s_d[i], path.s_dd[i],
                                                   path.d[i], d_prime, d_pprime);

            if (std::abs(cs.kappa) > limits.max_curvature) {
                path.valid = false;
                break;
            }
        }
    }
}

// =========================================================
// [Sec.VII] resolveManeuver
// =========================================================

// AVOID 모드는 PlannerCommand.avoidance_d_offset을 그대로 lateral 목표 중심으로 사용.
// LANE_CHANGE_*는 물리적 차선폭 파라미터가 아직 설정에 없어 TODO로 남김.
double ResolveLateralOffset(const PlannerCommand& cmd) {
    switch (cmd.mode) {
        case AVOID:
            return cmd.avoidance_d_offset;

        case LANE_CHANGE_LEFT:
        case LANE_CHANGE_RIGHT:
            // TODO(추후 개발 필요): lane_width 설정값이 config에 추가되면
            // target_lane * lane_width 로 목표 오프셋을 계산해야 함.
            // 지금은 차선 변경 오프셋을 반영하지 않고 0으로 둔다.
            return 0.0;

        default:
            return 0.0;  // 차선 중앙 유지
    }
}

std::vector<FrenetPath> ResolveManeuver(const FrenetState& start,
                                         const PlannerCommand& cmd,
                                         const RefLine& ref,
                                         const PathGeneratorConfig& cfg,
                                         const KinematicLimits& limits) {
    // 1. lateral 후보 (필요 시 목표 오프셋만큼 d1 격자를 평행이동)
    PathGeneratorConfig lateral_cfg = cfg;
    const double d_offset = ResolveLateralOffset(cmd);
    lateral_cfg.lateral_d1.min += d_offset;
    lateral_cfg.lateral_d1.max += d_offset;

    std::vector<FrenetPath> lateral_set = GenerateLateralCandidates(start, lateral_cfg);
    FilterLateralByAcceleration(lateral_set, limits);

    // 2. longitudinal 후보 (FSM이 지정한 모드에 따라 생성 방식 선택)
    std::vector<FrenetPath> longitudinal_set;

    switch (cmd.mode) {
        case FOLLOWING:
            longitudinal_set = GenerateFollowingCandidates(start, cmd.leader_s, cmd.leader_speed,
                                                            cmd.leader_accel, cmd.time_gap,
                                                            cmd.min_gap, cfg);
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

    // Merging(Sec.V-A)은 두 대상 차량(sa,sb) 정보가 필요한데, 현재 BehaviorState에는
    // 대응하는 상태가 없고 PlannerCommand도 leader 하나만 표현 가능함.
    // TODO(추후 개발 필요): FSM에 MERGE 상태 및 두 차량 정보 필드 추가 후 연동.

    FilterLongitudinalByAcceleration(longitudinal_set, limits);

    // 3. 결합 + 사후 곡률 필터링
    std::vector<FrenetPath> combined = CombineLateralLongitudinal(lateral_set, longitudinal_set);
    FilterByCurvature(combined, ref, limits);

    return combined;
}
