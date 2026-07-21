#include "frenet_planner/frenet_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "global/utils.hpp"

namespace {

// 곡률 -> 목표속도.
//
// 예전엔 곡률 구간(0.015/0.035/0.080)마다 고정된 속도(7.0/5.5/4.0)로 뚝 떨어지는
// 3단 계단식이었는데, 각 threshold에서 물리적으로 실제 낼 수 있는 속도
// (v=sqrt(max_lateral_accel/k))와 비교해보면 훨씬 낮았다 - 예를 들어
// k=0.08에서 물리적으로는 sqrt(3.0/0.08)=6.1m/s까지 가능한데 고정값 4.0으로
// 강제로 깎았다. 실측 재현 결과 이게 코너 앞에서 필요 이상으로 거의 멈추다시피
// 감속하고, 선회 중에도 "멈췄다 가는" 거동으로 이어져 오히려 추종을 방해했다
// (급감속/급가속이 반복되면서 헤딩이 못 따라가 d가 벌어지는 문제와도 연결).
// 계단 대신 물리 기반 연속 함수로 바꿔 곡률에 비례해 자연스럽게 감속하게 한다.
double VelocityFromCurvature(double k, const CurveSpeedConfig& cfg, double max_lateral_accel) {
    constexpr double kSafetyMargin = 0.8;         // 한계의 80%만 사용 (여유폭 확보)
    constexpr double kMinKappaForCalc = 1e-4;      // 0 나누기 방지 (거의 직선)
    const double v_curve = std::sqrt(max_lateral_accel * kSafetyMargin / std::max(k, kMinKappaForCalc));
    return clip(v_curve, 0.0, cfg.target_vel);
}

// 전방 lookahead 구간 내 최대 곡률로 미리 감속 (옛 lookaheadCurvature 이식).
//
// cfg.curve_lookahead_m(15m 고정)은 저속에서는 충분하지만, 고속(예: 9.5m/s로
// 진입)에서는 필요한 제동거리보다 짧아서 "곡선 진입 15m 전"에야 목표속도가
// 뚝 떨어지는데, 그 지점부터 급감속을 해도 다 못 줄이고 코너에 진입 -> 실제
// 곡률 대비 속도가 과도해 lateral 후보가 전부 무효화되는 문제가 실측으로
// 확인됐다(코너 진입 시 우회전 하려다 후보가 사라져 정지). 제동거리는 속도
// 제곱에 비례하므로(v0^2-v1^2)/(2a), 현재 속도 기준으로 그 거리를 계산해 15m와
// 비교해 더 큰 쪽을 쓴다. "가장 감속이 필요한 경우"의 목표속도로 max_curvature
// 지점에서의 물리적 최저속도를 쓴다(예전엔 curve_vel_sharp 고정값). 실제 궤적은
// jerk 비용 때문에 max_longitudinal_accel 전부를 안 쓰므로(더 부드러운 감속을
// 선호), 그 절반 정도만 "편하게 낼 수 있는 감속도"로 보수적으로 잡고 여유 거리도
// 더한다.
double LookaheadTargetSpeed(const RefLine& ref, double s_start, double ego_speed,
                             const CurveSpeedConfig& cfg, double max_longitudinal_accel,
                             double max_lateral_accel, double max_curvature) {
    constexpr double kComfortableDecelFactor = 0.5;  // 실제 평균 감속도는 max의 절반 정도로 가정
    constexpr double kReactionMargin = 5.0;          // [m] 추가 여유거리
    const double usable_decel = std::max(0.5, max_longitudinal_accel * kComfortableDecelFactor);
    const double v_floor = VelocityFromCurvature(max_curvature, cfg, max_lateral_accel);
    const double speed_drop = std::max(0.0, ego_speed * ego_speed - v_floor * v_floor);
    const double required_dist = speed_drop / (2.0 * usable_decel);
    const double lookahead_m = std::max(cfg.curve_lookahead_m, required_dist + kReactionMargin);

    double max_k = 0.0;
    for (const auto& p : ref.points) {
        if (p.s < s_start) continue;
        if (p.s > s_start + lookahead_m) break;
        max_k = std::max(max_k, std::fabs(p.kappa));
    }
    return VelocityFromCurvature(max_k, cfg, max_lateral_accel);
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

// 저속 fallback: 정지 근처에서 quintic 후보 생성/곡률필터를 거치지 않고,
// d(s)를 "s 기준(시간이 아니라 이동거리 기준)" 스무스스텝으로 직접 설계해
// FrenetToCartesian의 닫힌 형태로 바로 렌더링한다. d1을 그냥 고정해두는
// 기존 저속 처리(GenerateLateralCandidates 주석 참고)보다 적극적으로 d를
// 0으로 되돌리면서도, 두 가지를 특히 주의한다:
//
// 1) 시간 기준(예: "3초 안에 복귀")으로 되돌리면 저속일수록 같은 시간에
//    이동거리가 짧아져 오히려 더 급한 커브가 필요해진다(실측 재현: fallback을
//    처음 시간 기준으로 짜자 kappa가 -0.46까지 튀었음 - "느릴수록 더 급하게
//    꺾어야 하는" 역설). 이동거리(kRecenterDistM) 기준으로 바꾸면 속도와
//    무관하게 항상 같은 커브 완만도를 보장한다.
// 2) d(s)를 직선(선형)으로 램프하면 양 끝에서 d''(기울기의 변화)가 불연속으로
//    튀어 그 지점에서 곡률이 스파이크된다. cos 기반 smoothstep은 양끝에서
//    기울기(d')가 0으로 자연스럽게 붙어 이 스파이크가 없다.
bool FrenetPlanner::PlanLowSpeedFallback(const CartesianState& ego, double s, double d,
                                          CartesianPath& out_path) const {
    constexpr double kRecenterDistM = 15.0;   // [m] d를 0으로 되돌리는 데 쓰는 이동거리
    constexpr double kCrawlAccel = 0.5;       // [m/s^2] 완만한 가속
    constexpr double kMinCreepSpeed = 0.3;    // [m/s] 최소 전진속도 (0 근처에 갇히지 않도록)
    constexpr double kCrawlSpeedCap = 2.0;    // [m/s] 이 fallback 안에서의 속도 상한
    constexpr double kDurationS = 2.0;        // [s] 생성할 경로 길이

    const int n = static_cast<int>(kDurationS / path_cfg_.dt) + 1;
    out_path.x.resize(n); out_path.y.resize(n); out_path.yaw.resize(n);
    out_path.kappa.resize(n); out_path.v.resize(n); out_path.a.resize(n);

    double s_i = s;
    double v_prev = std::max(ego.v, kMinCreepSpeed);
    for (int i = 0; i < n; ++i) {
        const double t = i * path_cfg_.dt;
        const double v_i = clip(ego.v + kCrawlAccel * t, kMinCreepSpeed, kCrawlSpeedCap);
        if (i > 0) s_i += 0.5 * (v_i + v_prev) * path_cfg_.dt;
        v_prev = v_i;

        const double ds = s_i - s;
        const double frac = clip(ds / kRecenterDistM, 0.0, 1.0);
        // smoothstep(frac) = 0.5*(1+cos(pi*frac)): frac=0 -> d, frac=1 -> 0, 양끝 기울기 0.
        const double d_i      = d * 0.5 * (1.0 + std::cos(M_PI * frac));
        const double d_prime  = (ds >= kRecenterDistM) ? 0.0
                               : d * (-0.5 * M_PI / kRecenterDistM) * std::sin(M_PI * frac);
        const double d_pprime = (ds >= kRecenterDistM) ? 0.0
                               : d * (-0.5 * M_PI * M_PI / (kRecenterDistM * kRecenterDistM))
                                     * std::cos(M_PI * frac);

        RefPoint rp = Interpolate(ref_, s_i);
        const double a_i = (v_i < kCrawlSpeedCap) ? kCrawlAccel : 0.0;
        CartesianState cs = FrenetToCartesian(rp, s_i, v_i, a_i, d_i, d_prime, d_pprime);

        out_path.x[i] = cs.x;
        out_path.y[i] = cs.y;
        out_path.yaw[i] = cs.yaw;
        out_path.kappa[i] = cs.kappa;
        out_path.v[i] = v_i;
        out_path.a[i] = a_i;
    }
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

    // 저속(정지 근처)에서는 quintic 후보/곡률필터 파이프라인 전체를 건너뛰고
    // fallback으로 직접 경로를 만든다 (클래스 헤더 주석 참고 - 나누기 없는 안전한
    // 경로). AVOID/장애물 회피는 이 fallback에서는 다루지 않는다 - 정지 근처
    // 속도로는 위험한 회피 기동이 필요할 상황이 원래 적고, 속도가 오르면 바로
    // 아래의 정상 파이프라인으로 자동 복귀한다.
    last_d_ = d;

    constexpr double kLowSpeedFallbackV = 0.5;  // [m/s]
    if (std::fabs(ego.v) < kLowSpeedFallbackV) {
        return PlanLowSpeedFallback(ego, s, d, out_path);
    }

    double d_dot, d_ddot;
    ArcDerivToTimeDeriv(s_dot, s_ddot, d_prime, d_pprime, d_dot, d_ddot);
    last_d_dot_ = d_dot;  // 클램프 전 원본 값 - 진단 목적(아래 kMaxDDot 클램프 폭 자체를 보려고)

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

    // d_dot(횡방향 속도)도 같은 이유로 클램프가 필요하다: d_prime = (1-kr*d)*tan(delta_theta)라
    // delta_theta(실제 yaw와 참조선 heading의 차이)가 커지면 d_dot=s_dot*d_prime도
    // 같이 커진다. 실측 재현: 컨트롤이 흔들리던 구간에서 d_dot=-1.19m/s까지 커진
    // 상태로 넘어오자, "중앙(d1=0)으로 되돌리는" 후보조차 그 속도를 t=0에서 그대로
    // 이어받아야 해서(물리적 연속성) 초반에 오히려 더 벗어나야 했고, 그 결과 곡률이
    // 폭증해 289개 후보 전부가 무효화 -> 정지까지 이어졌다(d=-2.69에서 -2.96까지
    // 더 벗어난 뒤에야 멈춤). 정상 추종 중에는 d_dot이 0.1m/s 미만인 게 대부분이라
    // (실측 다수 확인), 이 범위를 크게 넘으면 "헤딩 추종 오차로 인한 과대추정"으로
    // 보고 s_ddot/d_ddot과 같은 방식으로 안전하게 클램프한다.
    constexpr double kMaxDDot = 0.5;  // [m/s]
    d_dot = clip(d_dot, -kMaxDDot, kMaxDDot);

    FrenetState start{s, s_dot, s_ddot, d, d_dot, d_ddot};

    // FSM 미완성: LANE_KEEPING 고정 + 곡률 기반 사전 감속만 반영.
    // AVOID 트리거/SAT 충돌필터(FilterByCollision, collision_checker.cpp)는
    // FSM이 붙기 전까지 비활성화 - obstacles는 그 검증 때만 쓰였고 지금은
    // 파라미터로만 받아둔 채 쓰지 않는다. SAT 로직 자체는 그대로 남아있으니
    // FSM 연동 시 여기서 다시 호출하면 된다.
    (void)obstacles;
    PlannerCommand cmd{};
    cmd.mode = LANE_KEEPING;
    cmd.target_speed = LookaheadTargetSpeed(ref_, s, ego.v, curve_speed_cfg_,
                                             limits_.max_longitudinal_accel,
                                             limits_.max_lateral_accel, limits_.max_curvature);

    PlannerDebugStats stats;
    std::vector<FrenetPath> candidates =
        ResolveManeuver(start, cmd, ref_, path_cfg_, limits_, lane_width_, &stats);

    EvaluateCosts(candidates, cost_weights_);

    const FrenetPath* best = SelectBestPath(candidates);
    if (!best) {
        std::printf("[FrenetPlanner] No valid candidate this cycle (%zu generated)\n", candidates.size());
        std::printf("[FrenetPlanner-DEBUG] ego: x=%.3f y=%.3f yaw=%.3f v=%.3f a=%.3f\n",
                     ego.x, ego.y, ego.yaw, ego.v, ego.a);
        std::printf("[FrenetPlanner-DEBUG] start: s=%.2f d=%.3f s_dot=%.2f s_ddot=%.2f d_dot=%.3f d_ddot=%.3f "
                     "target_speed=%.2f\n",
                     s, d, s_dot, s_ddot, d_dot, d_ddot, cmd.target_speed);
        std::printf("[FrenetPlanner-DEBUG] lateral %zu/%zu valid | longitudinal %zu/%zu valid | "
                     "combined %zu -> after curvature %zu valid\n",
                     stats.lateral_valid, stats.lateral_total,
                     stats.longitudinal_valid, stats.longitudinal_total,
                     stats.combined_total, stats.combined_valid_after_curvature);
        return false;
    }

    out_path = ConvertToCartesianPath(*best, ref_);
    return true;
}
