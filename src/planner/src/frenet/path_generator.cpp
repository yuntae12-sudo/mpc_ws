#include "frenet/path_generator.hpp"
#include "math/polynomial.hpp"
#include "math/frenet_converter.hpp"

#include <ros/ros.h>

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
                                                    const PathGeneratorConfig& cfg,
                                                    const KinematicLimits& limits) {
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

    // Creep 후보: stop_s가 현재 속도로 물리적으로 도달 불가능한 지점(격자 전부
    // FilterLongitudinalByAcceleration에 걸릴 만큼 가까움)이면, 한계 안에서
    // 가장 빨리 멈출 수 있는 지점을 target으로 대신 써서 최소 1개는 항상 남긴다.
    //
    // 거리를 T와 무관하게 하나로 고정하면 안 된다(수치 검증으로 확인된 버그,
    // 2026-07) - 예: v=6.91, margin=0.5 기준 고정거리 15.92m를 T=2s로 가려면
    // 첨두감속도가 16.6m/s²까지 치솟는다(한계의 5배+). T가 짧을수록 같은 거리를
    // 가는 데 필요한 감속이 기하급수적으로 커지기 때문 - GenerateVelocityKeepingCandidates의
    // creep(T별 max_dv)과 똑같이 T마다 개별 계산해야 한다.
    //
    // xe(T) = start.s + start.s_d*T/2 는 "이 T 안에 v->0으로 등감속 정지하는"
    // 거리이고, 이 형태(시작/끝 가속도=0)의 quintic은 이 거리에서 첨두감속도가
    // *최소*가 된다(첨두/평균 비율 1.5, quartic과 동일 - 수치 검증됨). 이 거리보다
    // 가깝든 멀든 벗어나면 첨두감속도가 더 커진다(가까우면 급정지, 멀면 뒤늦게
    // 급브레이크) - 그래서 stop_s와 비교해서 "더 먼 쪽"을 고르면 안 된다
    // (실제로 그렇게 했다가 stop_s가 자연거리보다 먼 경우에도 첨두가속도
    // 초과로 creep이 죽는 버그가 실측으로 확인됨, 2026-07). creep은 stop_s가
    // 얼마든 항상 이 자연거리 하나만 목표로 한다 - "한계 안에서 낼 수 있는
    // 가장 안전한 정지"가 목적이지 stop_s에 최대한 가까이 가는 게 목적이 아님.
    //
    // 평균감속도 start.s_d/T가 margin*max_longitudinal_accel(=peak/1.5 한계)
    // 이하인 T에서만 creep을 만든다 - 그보다 짧은 T는 이 속도로는 애초에
    // 안전하게 못 서는 시간대라 어떤 목표를 잡아도 못 구하므로 생성을 건너뛴다.
    const double avg_decel_limit = limits.max_longitudinal_accel * limits.creep_accel_margin;
    for (double T : T_list) {
        if (T <= 1e-6) continue;
        if (avg_decel_limit > 1e-9 && start.s_d / T > avg_decel_limit) continue;

        std::vector<double> times = SampleTimes(T, cfg.dt);
        const double s1_creep = start.s + start.s_d * T / 2.0;
        QuinticPolynomial poly = MakeQuintic(start.s, start.s_d, start.s_dd,
                                              s1_creep, 0.0, 0.0, T);
        FrenetPath path = SampleLongitudinalQuintic(poly, times);
        path.delta_s = s1_creep - stop_s;

        result.push_back(std::move(path));
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
// [Sec.IV-B] 저속 모드 — d(s) quintic (설계 노트는 헤더 참고)
// =========================================================

std::vector<FrenetPath> GenerateLowSpeedCombinedCandidates(const FrenetState& start,
                                                            const std::vector<FrenetPath>& longitudinal_set,
                                                            const PathGeneratorConfig& cfg,
                                                            const KinematicLimits& limits) {
    std::vector<FrenetPath> result;
    std::vector<double> d1_list = SampleRange(cfg.lateral_d1);

    for (const auto& lon : longitudinal_set) {
        if (!lon.valid || lon.s.empty()) continue;

        const double delta_S = lon.s.back() - start.s;

        if (delta_S <= 1e-6) {
            // 이동거리가 사실상 0(정지 유지) - quintic을 만들 수 없으므로
            // 현재 d를 그대로 유지하는 단일 후보만 추가한다.
            FrenetPath combined;
            combined.t = lon.t;
            combined.s = lon.s;
            combined.s_d = lon.s_d;
            combined.s_dd = lon.s_dd;
            combined.d.assign(lon.t.size(), start.d);
            combined.d_d.assign(lon.t.size(), 0.0);
            combined.d_dd.assign(lon.t.size(), 0.0);
            combined.valid = true;
            combined.cost_lat = combined.cost_lon = combined.cost_total = 0.0;
            combined.jerk_cost_lat = 0.0;
            combined.jerk_cost_lon = lon.jerk_cost_lon;
            combined.delta_s = lon.delta_s;
            combined.delta_s_dot = lon.delta_s_dot;
            result.push_back(std::move(combined));
            continue;
        }

        for (double d1 : d1_list) {
            // 종료조건 [d1, 0, 0, ΔS] — 고속과 동일하게 "차선과 평행하게 복귀"를
            // 목표로 하되, 매개변수가 시간이 아니라 이 후보가 실제로 커버하는
            // 호길이(ΔS)라는 점만 다르다 (Sec.IV-B).
            QuinticPolynomial poly_d = MakeQuintic(start.d, start.d_prime, start.d_pprime,
                                                    d1, 0.0, 0.0, delta_S);

            FrenetPath combined;
            combined.t = lon.t;
            combined.s = lon.s;
            combined.s_d = lon.s_d;
            combined.s_dd = lon.s_dd;
            combined.valid = true;
            combined.cost_lat = combined.cost_lon = combined.cost_total = 0.0;
            combined.jerk_cost_lat = JerkCost(poly_d, delta_S);  // Jt[d(s)] = ∫d'''(s)^2 ds
            combined.jerk_cost_lon = lon.jerk_cost_lon;
            combined.delta_s = lon.delta_s;
            combined.delta_s_dot = lon.delta_s_dot;

            combined.d.reserve(lon.s.size());
            combined.d_d.reserve(lon.s.size());
            combined.d_dd.reserve(lon.s.size());

            bool lateral_accel_ok = true;
            for (size_t i = 0; i < lon.s.size(); i++) {
                const double s_local  = lon.s[i] - start.s;  // quintic은 Δs=0 기준으로 정의됨
                const double d_val    = EvalPos(poly_d, s_local);
                const double d_prime  = EvalVel(poly_d, s_local);
                const double d_pprime = EvalAcc(poly_d, s_local);

                // ArcDerivToTimeDeriv는 s_dot을 곱하기만 하므로 s_dot=0(정지)에서도 안전
                // (TimeDerivToArcDeriv의 역방향 - 나눗셈 - 은 절대 쓰지 않는다).
                double d_dot, d_ddot;
                ArcDerivToTimeDeriv(lon.s_d[i], lon.s_dd[i], d_prime, d_pprime, d_dot, d_ddot);

                // i==0(t=0)은 어떤 d1을 고르든 항상 start의 실측 상태와 같은 값이라
                // 후보 판별에 정보가 없다 - FilterLateralByAcceleration과 동일한 이유로
                // 건너뛴다(2026-07-10).
                if (i > 0 && std::abs(d_ddot) > limits.max_lateral_accel) lateral_accel_ok = false;

                combined.d.push_back(d_val);
                combined.d_d.push_back(d_dot);
                combined.d_dd.push_back(d_ddot);
            }

            combined.valid = lateral_accel_ok;
            result.push_back(std::move(combined));
        }
    }

    return result;
}

// =========================================================
// [Sec.V-B] Velocity Keeping
// =========================================================

std::vector<FrenetPath> GenerateVelocityKeepingCandidates(const FrenetState& start,
                                                           double target_speed,
                                                           const PathGeneratorConfig& cfg,
                                                           const KinematicLimits& limits) {
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

        // Creep 후보: target_speed가 현재 속도와 너무 벌어져서(예: FSM 링크 전환
        // 직후) 격자 전부가 FilterLongitudinalByAcceleration에 걸려도, 이 T 안에
        // 한계 가속도로 갈 수 있는 만큼만 목표로 하는 후보를 항상 남긴다.
        const double max_dv = limits.max_longitudinal_accel * limits.creep_accel_margin * T;
        const double v1_creep = start.s_d + std::min(std::max(target_speed - start.s_d, -max_dv), max_dv);
        QuarticPolynomial creep_poly = MakeQuartic(start.s, start.s_d, start.s_dd,
                                                    v1_creep, 0.0, T);
        FrenetPath creep_path = SampleLongitudinalQuartic(creep_poly, times);
        creep_path.delta_s_dot = v1_creep - target_speed;
        result.push_back(std::move(creep_path));
    }

    return result;
}

// =========================================================
// [Sec.VI 1단] 결합 전 가속도 필터링
// =========================================================

// index 0(t=0)은 어떤 후보를 고르든 항상 start.d_dd/start.s_dd(실측 현재
// 가속도)와 동일한 값이라 후보 간 구분에 아무 정보가 없다 - 그런데 실측
// 가속도가 한계를 넘는 순간(예: MPC의 급브레이크 직후, accel_min=-5.0이
// planner의 max_longitudinal_accel=3.0보다 더 넓은 범위) 모든 후보가 target과
// 무관하게 t=0에서부터 몰살당해 candidates가 통째로 비어버리는 버그가 실측으로
// 확인됨(2026-07-10). "이 후보가 계획한 미래"만 검사해야 하므로 t=0은 건너뛴다.
void FilterLateralByAcceleration(std::vector<FrenetPath>& candidates,
                                  const KinematicLimits& limits) {
    for (auto& path : candidates) {
        if (!path.valid) continue;
        for (size_t i = 1; i < path.d_dd.size(); ++i) {
            if (std::abs(path.d_dd[i]) > limits.max_lateral_accel) {
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
        for (size_t i = 1; i < path.s_dd.size(); ++i) {
            if (std::abs(path.s_dd[i]) > limits.max_longitudinal_accel) {
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
        for (size_t i = 0; i < geo.kappa.size(); i++) {
            // kMinSegmentLength(위치 기반 fallback)로는 못 잡는다 - 저속에서
            // 노이즈가 "특정 거리 이하로 뚝 끊기는" 게 아니라 속도가 낮을수록
            // 연속적으로 커진다(실측: seg_len=0.2~0.28m, kappa=0.18~0.37로 여전히
            // 한계 초과, 임계값을 올려도 경계가 옮겨질 뿐). 세그먼트 길이 같은
            // 간접 지표 대신, 정확히 아는 실제 속도로 직접 판단한다 - 이 속도
            // 미만(Sec.IV-B의 kLowSpeedThreshold)에서는 위치기반 곡률 추정 자체가
            // 원래 신뢰 불가능하므로(코드베이스 전반의 기존 전제) 검사를 건너뛴다.
            if (std::abs(path.s_d[i]) < kLowSpeedThreshold) continue;
            if (std::abs(geo.kappa[i]) > limits.max_curvature) {
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
// LANE_CHANGE_*는 d 양의 방향이 좌측(FrenetToCartesian의 n_r 정의와 동일)이므로
// LEFT는 +lane_width, RIGHT는 -lane_width.
double ResolveLateralOffset(const PlannerCommand& cmd, double lane_width) {
    switch (cmd.mode) {
        case AVOID:
            return cmd.avoidance_d_offset;

        case LANE_CHANGE_LEFT:
            return lane_width;

        case LANE_CHANGE_RIGHT:
            return -lane_width;

        default:
            return 0.0;  // 차선 중앙 유지
    }
}

std::vector<FrenetPath> ResolveManeuver(const FrenetState& start,
                                         const PlannerCommand& cmd,
                                         const RefLine& ref,
                                         const PathGeneratorConfig& cfg,
                                         const KinematicLimits& limits,
                                         double lane_width) {
    // 1. lateral 격자 (필요 시 목표 오프셋만큼 d1 격자를 평행이동) - 고속/저속
    //    양쪽 다 이 오프셋 적용된 lateral_cfg.lateral_d1을 그대로 쓴다.
    PathGeneratorConfig lateral_cfg = cfg;
    const double d_offset = ResolveLateralOffset(cmd, lane_width);
    lateral_cfg.lateral_d1.min += d_offset;
    lateral_cfg.lateral_d1.max += d_offset;

    // STOP/EMERGENCY/INTERSECTION_WAIT은 차선변경을 시도할 이유가 없는데도
    // 고속 lateral 격자(±3m, 시간 기준 d(t))를 그대로 쓰면 문제가 생긴다 - 종방향이
    // T 후반부로 갈수록 속도 0으로 수렴해 호길이 진행이 거의 없어지는데, 횡방향은
    // 여전히 "그 T 안에 -3~+3m를 다 이동"하려고 해서 (x,y) 상에서 거의 제자리로
    // 확 꺾이는 기하학적 곡선이 만들어지고 FilterByCurvature가 전부 무효화시킨다
    // (실차 검증으로 확인, 2026-07 - 같은 도로 구간이 LANE_KEEPING일 땐 문제없어서
    // 도로 곡률이 아니라 이 결합 방식이 원인임을 확인함). 정지 중 차선유지가
    // 당연하므로 d1 격자를 지금 위치(start.d) 하나로 좁혀 이 결합 자체를 없앤다.
    if (cmd.mode == STOP || cmd.mode == EMERGENCY || cmd.mode == INTERSECTION_WAIT) {
        lateral_cfg.lateral_d1.min = start.d;
        lateral_cfg.lateral_d1.max = start.d;
        lateral_cfg.lateral_d1.step = 0.0;
    }

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
            longitudinal_set = GenerateStoppingCandidates(start, cmd.stop_position, cfg, limits);
            break;

        case EMERGENCY:
            // cmd.stop_position은 FSM이 정지선 등으로 이미 알고 있으면 그 값,
            // 모르면(TTC/emergency_risk 트리거) behavior_bridge.cpp가 현재 속도와
            // max_longitudinal_accel로 직접 계산한 물리 기반 정지거리다 - FSM은
            // "EMERGENCY 상황"만 판단하고 "어디서 멈출지"는 planner 책임.
            longitudinal_set = GenerateStoppingCandidates(start, cmd.stop_position, cfg, limits);
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
            longitudinal_set = GenerateVelocityKeepingCandidates(start, cmd.target_speed, cfg, limits);
            break;
    }

    // Merging(Sec.V-A)은 두 대상 차량(sa,sb) 정보가 필요한데, 현재 BehaviorState에는
    // 대응하는 상태가 없고 PlannerCommand도 leader 하나만 표현 가능함.
    // TODO(추후 개발 필요): FSM에 MERGE 상태 및 두 차량 정보 필드 추가 후 연동.

    FilterLongitudinalByAcceleration(longitudinal_set, limits);

    // 3. lateral 생성 + 결합 (Sec.IV-B: 저속은 d(s), 고속은 d(t) - 헤더 설계 노트 참고)
    std::vector<FrenetPath> combined;
    const bool low_speed = std::abs(start.s_d) < kLowSpeedThreshold;
    if (low_speed) {
        combined = GenerateLowSpeedCombinedCandidates(start, longitudinal_set, lateral_cfg, limits);
    } else {
        std::vector<FrenetPath> lateral_set = GenerateLateralCandidates(start, lateral_cfg);

        // 진단용(임시): 횡방향 후보가 전멸하는 게 "속도가 오를수록 start.d_dd
        // (CartesianToFrenet의 s_dot^2 스케일링이 들어간 경계조건)가 부풀어서"인지
        // 확인한다 - lateral_peak_ddot는 필터링 전, 후보들 전체에서 관측된
        // |d_dd| 최댓값.
        double lateral_peak_ddot = 0.0;
        for (const auto& p : lateral_set)
            for (double dd : p.d_dd) lateral_peak_ddot = std::max(lateral_peak_ddot, std::abs(dd));

        FilterLateralByAcceleration(lateral_set, limits);

        int lateral_survivors = 0;
        for (const auto& p : lateral_set) if (p.valid) lateral_survivors++;
        if (lateral_survivors == 0) {
            ROS_WARN_THROTTLE(0.2, "[LateralAccelDiag] start.d=%.3f d_d=%.3f d_dd=%.3f s_d=%.2f "
                                    "lateral_peak_ddot=%.3f limit=%.2f",
                               start.d, start.d_d, start.d_dd, start.s_d,
                               lateral_peak_ddot, limits.max_lateral_accel);
        }

        combined = CombineLateralLongitudinal(lateral_set, longitudinal_set);
    }

    // 4. 결합 이후 곡률 필터링
    FilterByCurvature(combined, ref, limits);

    return combined;
}
