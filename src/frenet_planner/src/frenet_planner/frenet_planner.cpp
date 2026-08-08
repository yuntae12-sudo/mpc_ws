#include "frenet_planner/frenet_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iterator>
#include <limits>

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

    // 폐루프 wrap-around: s_start+lookahead_m이 s_max를 넘으면(피니시라인
    // 근처) 원래 코드는 뒤가 안 나와 break만 되고 이음새 너머(실제로는 바로
    // 이어지는 트랙 시작 구간)의 곡률을 놓쳤다 - 그 구간에 급커브가 있으면
    // 감속 없이 그대로 진입하게 된다. s_max로 감아서 [s_start, s_max]와
    // [0, 넘친 만큼] 두 구간을 모두 본다.
    const double s_max = ref.points.back().s;
    double s_start_w = std::fmod(s_start, s_max);
    if (s_start_w < 0.0) s_start_w += s_max;
    const double s_end_w = s_start_w + lookahead_m;
    const double wrapped_end = s_end_w - s_max;  // s_end_w > s_max일 때만 유효

    double max_k = 0.0;
    for (const auto& p : ref.points) {
        const bool in_range = (s_end_w <= s_max)
            ? (p.s >= s_start_w && p.s <= s_end_w)
            : (p.s >= s_start_w || p.s <= wrapped_end);
        if (in_range) max_k = std::max(max_k, std::fabs(p.kappa));
    }
    return VelocityFromCurvature(max_k, cfg, max_lateral_accel);
}

const char* ModeName(BehaviorState mode) {
    switch (mode) {
        case LANE_KEEPING: return "LANE_KEEPING";
        case FOLLOWING: return "FOLLOWING";
        case AVOID: return "AVOID";
        default: return "OTHER";
    }
}

const char* AvoidPhaseName(int phase) {
    switch (phase) {
        case 0: return "TRACKING";
        case 1: return "SHIFT";
        case 2: return "PASS";
        default: return "NONE";
    }
}

// 후보 경로의 최대 기하 곡률에서 a_lat=v^2*|kappa| 한계를
// 만족하는 속도를 계산한다. 제어/모델 오차 여유로 한계의 80%를 쓴다.
double CandidateCurvatureSpeedLimit(const FrenetPath& path, const RefLine& ref,
                                    double max_lateral_accel) {
    constexpr double kLateralAccelMargin = 0.8;
    constexpr double kMinCurvature = 1e-4;
    const GeometricPath geo = ComputeGeometricPath(path.s, path.d, ref);
    double max_kappa = 0.0;
    for (double kappa : geo.kappa) max_kappa = std::max(max_kappa, std::fabs(kappa));
    if (max_kappa < kMinCurvature) return std::numeric_limits<double>::infinity();
    return std::sqrt(max_lateral_accel * kLateralAccelMargin / max_kappa);
}

// 재생성된 후보의 실제 샘플별 속도²×곡률을 검증한다. 단순히
// 기하 곡률만 작은 경로가 아니라, 해당 속도에서 실제로 횡가속도
// 한계를 만족하는 경로만 남긴다.
void FilterByCartesianLateralAcceleration(std::vector<FrenetPath>& candidates,
                                          const RefLine& ref,
                                          double max_lateral_accel) {
    constexpr double kLateralAccelMargin = 0.8;
    const double limit = max_lateral_accel * kLateralAccelMargin;
    for (auto& path : candidates) {
        if (!path.valid) continue;
        const GeometricPath geo = ComputeGeometricPath(path.s, path.d, ref);
        const size_t count = std::min(geo.kappa.size(), path.s_d.size());
        for (size_t i = 0; i < count; ++i) {
            const double lateral_accel = path.s_d[i] * path.s_d[i] * std::fabs(geo.kappa[i]);
            if (lateral_accel > limit) {
                path.valid = false;
                path.rejection_reason = RejectionReason::CURVATURE;
                break;
            }
        }
    }
}

}  // namespace

bool FrenetPlanner::Init(const std::string& yaml_path) {
    std::string waypoint_file;
    LoadParams(yaml_path, path_cfg_, limits_, cost_weights_, vehicle_shape_,
               collision_cfg_, curve_speed_cfg_, following_cfg_, avoid_cfg_,
               visualization_cfg_,
               wheelbase_, lane_width_, waypoint_file);

    debug_writer_ = std::make_unique<PlannerDebugWriter>(visualization_cfg_);

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
                                          double target_d,
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
        const double d_error = d - target_d;
        const double d_i      = target_d + d_error * 0.5 * (1.0 + std::cos(M_PI * frac));
        const double d_prime  = (ds >= kRecenterDistM) ? 0.0
                               : d_error * (-0.5 * M_PI / kRecenterDistM) * std::sin(M_PI * frac);
        const double d_pprime = (ds >= kRecenterDistM) ? 0.0
                               : d_error * (-0.5 * M_PI * M_PI / (kRecenterDistM * kRecenterDistM))
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
        static int low_speed_log_tick = 0;
        if (++low_speed_log_tick % 5 == 0) {
            std::printf("[FrenetPlanner-STATE] mode=LOW_SPEED_FALLBACK "
                        "s=%.2f d=%.3f speed=%.2f candidates=N/A\n",
                        s, d, ego.v);
        }
        // TRACKING 근거리에서 한 점짜리 v=0 경로를 무기한 보내던 분기는
        // ego.v==0 -> 같은 조기 return이라는 자기잠금을 만들었다. 정적 장애물을
        // 등록할 때 검증한 1차 회피 방향을 저속 경로의 목표로 사용해 재출발한다.
        // 정상 속도로 회복되면 기존 좌/우 전체 후보 비교가 방향을 다시 확정한다.
        const bool tracking_near = avoidance_.active &&
            avoidance_.phase == AvoidPhase::TRACKING &&
            avoidance_.obstacle_s - s <= avoid_cfg_.shift_start_distance;
        const bool hold_avoid_target = avoidance_.active &&
            (tracking_near || avoidance_.phase == AvoidPhase::SHIFT ||
             avoidance_.phase == AvoidPhase::PASS);
        const double fallback_target_d = hold_avoid_target ? avoidance_.target_d : 0.0;
        return PlanLowSpeedFallback(ego, s, d, fallback_target_d, out_path);
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

    // Behavior Planner 연동 전의 정적 장애물 AVOID context. 장애물 ID와
    // 회피 방향을 최초 1회만 고정한다. TRACKING 동안은 LANE_KEEPING을
    // 유지하고, 실제 기동 거리에서 SHIFT->PASS를 AVOID 안에서 수행한다.
    // 장애물 통과 후 중앙 복귀는 중복된 RETURN 단계 대신 LANE_KEEPING이 담당한다.
    PlannerCommand cmd{};
    double avoid_offset = 0.0;
    double leader_s = 0.0, leader_speed = 0.0, leader_accel = 0.0;
    bool begin_avoidance = false;

    // 활성 context의 장애물 위치를 새 UDP 프레임으로 갱신한다. 일시적
    // packet dropout이 있어도 저장된 위치/길이로 회피를 계속한다.
    if (avoidance_.active) {
        for (const auto& obj : obstacles) {
            if (obj.id != avoidance_.obstacle_id) continue;
            double obj_s, obj_d, obj_s_dot;
            ProjectObjectToFrenet(ref_, obj, obj_s, obj_d, obj_s_dot);
            avoidance_.obstacle_s = obj_s;
            avoidance_.obstacle_d = obj_d;
            avoidance_.obstacle_width = obj.width;
            avoidance_.obstacle_length = obj.length;
            break;
        }

        const double gap = avoidance_.obstacle_s - s;
        const double clear_distance = 0.5 * (vehicle_shape_.length + avoidance_.obstacle_length)
                                    + avoid_cfg_.pass_clearance;
        if (avoidance_.phase == AvoidPhase::TRACKING && gap <= avoid_cfg_.shift_start_distance) {
            // 이 사이클에서 좌/우 전체 후보를 비교한 뒤, 유효 경로가
            // 있는 방향만 SHIFT로 잠그다.
            begin_avoidance = true;
        } else if (avoidance_.phase == AvoidPhase::SHIFT &&
                   std::fabs(d - avoidance_.target_d) <= avoid_cfg_.lateral_tolerance) {
            avoidance_.phase = AvoidPhase::PASS;
            std::printf("[FrenetPlanner] avoid phase: SHIFT -> PASS (id=%d gap=%.2f)\n",
                        avoidance_.obstacle_id, gap);
        } else if ((avoidance_.phase == AvoidPhase::SHIFT || avoidance_.phase == AvoidPhase::PASS) &&
                   gap < -clear_distance) {
            // 자차 뒤가 장애물을 완전히 통과했으므로 AVOID를 즉시 종료한다.
            // 이 아래의 !avoidance_.active 탐색이 같은 사이클에서 다음 정적
            // 장애물을 TRACKING으로 등록하고, 중앙 복귀는 LANE_KEEPING이 수행한다.
            std::printf("[FrenetPlanner] avoid complete after PASS: id=%d gap=%.2f\n",
                        avoidance_.obstacle_id, gap);
            avoidance_ = AvoidanceContext{};
        }
    }

    if (!avoidance_.active) {
        StaticObstacleTarget target;
        // LANE_KEEPING 충돌 필터는 reactive_lookahead 시간까지 미리 본다.
        // 정적 장애물 추적 거리가 그보다 짧으면, 장애물을 TRACKING으로
        // 등록하기 전에 충돌 필터가 중앙 후보를 전부 제거 -> stop path ->
        // 등록 후 다시 가속하는 속도 급락/반등이 생긴다. 설정값은 최소
        // 탐지거리로 쓰고, 실제 등록 거리는 "최고속도×충돌 lookahead +
        // 차량 길이/마진"보다 항상 길게 잡는다.
        AvoidConfig acquisition_cfg = avoid_cfg_;
        const double collision_horizon_distance =
            curve_speed_cfg_.target_vel * collision_cfg_.reactive_lookahead
            + vehicle_shape_.length + collision_cfg_.safety_margin;
        acquisition_cfg.detection_distance =
            std::max(avoid_cfg_.detection_distance, collision_horizon_distance);
        if (FindStaticAvoidanceTarget(ref_, start, obstacles, lane_width_, vehicle_shape_,
                                      acquisition_cfg, target)) {
            avoidance_.active = true;
            avoidance_.obstacle_id = target.id;
            avoidance_.phase = AvoidPhase::TRACKING;
            avoidance_.obstacle_s = target.s;
            avoidance_.obstacle_d = target.d;
            avoidance_.obstacle_width = target.width;
            avoidance_.obstacle_length = target.length;
            // 저속/정지 상태에서도 영구 정지하지 않도록 selector가 확인한
            // 1차 방향을 보관한다. 정상 속도에서는 회피 시작 직전 양쪽의
            // 전체 후보를 비교해 더 정확한 target_d로 덮어쓴다.
            avoidance_.target_d = target.avoidance_offset;
            std::printf("[FrenetPlanner] static obstacle tracked: id=%d gap=%.2f "
                        "effective_detection=%.2f\n",
                        target.id, target.s - s, acquisition_cfg.detection_distance);
        }
    }

    // FOLLOWING 탐색거리도 충돌 lookahead보다 항상 길게 잡아,
    // leader를 모드 전환 전에 충돌물로만 먼저 보고 정지하는 현상을 막는다.
    FollowingConfig effective_following_cfg = following_cfg_;
    const double following_collision_horizon =
        curve_speed_cfg_.target_vel * collision_cfg_.reactive_lookahead
        + vehicle_shape_.length + collision_cfg_.safety_margin;
    effective_following_cfg.max_leader_search_s =
        std::max(following_cfg_.max_leader_search_s, following_collision_horizon);

    // 직전 leader ID를 유지해 탐색거리 경계/패킷 누락에서 모드가
    // FOLLOWING<->LANE_KEEPING으로 번복되지 않게 한다.
    if (following_.active) {
        bool id_seen = false;
        bool leader_valid = false;
        const double track_length = ref_.points.empty() ? 0.0 : ref_.points.back().s;
        for (const auto& obj : obstacles) {
            if (obj.id != following_.leader_id) continue;
            id_seen = true;
            double obj_s, obj_d, obj_s_dot;
            ProjectObjectToFrenet(ref_, obj, obj_s, obj_d, obj_s_dot);
            double gap = obj_s - s;
            if (gap <= 0.0 && track_length > 0.0) gap += track_length;
            const bool same_direction = obj_s_dot > following_cfg_.min_leader_speed;
            const bool same_lane = std::fabs(obj_d) <= lane_width_ * 0.5;
            const bool in_range = gap > 0.0 &&
                gap <= effective_following_cfg.max_leader_search_s
                     + following_cfg_.exit_search_margin;
            if (same_direction && same_lane && in_range) {
                following_.leader_s = s + gap;
                following_.leader_d = obj_d;
                following_.leader_speed = obj_s_dot;
                following_.leader_accel = 0.0;
                following_.missing_cycles = 0;
                leader_valid = true;
            }
            break;
        }

        if (!leader_valid) {
            if (id_seen) {
                // 관측됐지만 차선/방향/범위가 바뀐 것은 dropout이 아니므로 즉시 해제.
                following_ = FollowingContext{};
            } else if (++following_.missing_cycles <= following_cfg_.dropout_grace_cycles) {
                // ObjectInfo 일시 누락 동안은 직전 속도로 leader s를 예측.
                constexpr double kPlannerCycleDt = 0.05;  // main.cpp 20Hz
                following_.leader_s += following_.leader_speed * kPlannerCycleDt;
            } else {
                following_ = FollowingContext{};
            }
        }
    }

    if (!following_.active) {
        LeaderTarget leader;
        if (FindLeader(ref_, start, obstacles, lane_width_, effective_following_cfg, leader)) {
            following_.active = true;
            following_.leader_id = leader.id;
            following_.leader_s = leader.s;
            following_.leader_d = leader.d;
            following_.leader_speed = leader.speed;
            following_.leader_accel = leader.accel;
            following_.missing_cycles = 0;
            std::printf("[FrenetPlanner] leader acquired: id=%d gap=%.2f speed=%.2f "
                        "effective_search=%.2f\n",
                        leader.id, leader.s - s, leader.speed,
                        effective_following_cfg.max_leader_search_s);
        }
    }

    if (begin_avoidance || (avoidance_.active && avoidance_.phase != AvoidPhase::TRACKING)) {
        cmd.mode = AVOID;
        avoid_offset = (avoidance_.phase == AvoidPhase::SHIFT || avoidance_.phase == AvoidPhase::PASS)
                     ? avoidance_.target_d : 0.0;
        cmd.avoidance_d_offset = avoid_offset;
    } else if (following_.active) {
        leader_s = following_.leader_s;
        leader_speed = following_.leader_speed;
        leader_accel = following_.leader_accel;
        cmd.mode = FOLLOWING;
        cmd.leader_id = following_.leader_id;
        cmd.leader_s = leader_s;
        cmd.leader_speed = leader_speed;
        cmd.leader_accel = leader_accel;
        cmd.time_gap = following_cfg_.time_gap;
        cmd.min_gap = following_cfg_.min_gap;
        cmd.gap_gain = following_cfg_.gap_gain;
    } else {
        cmd.mode = LANE_KEEPING;
    }

    cmd.target_speed = LookaheadTargetSpeed(ref_, s, ego.v, curve_speed_cfg_,
                                             limits_.max_longitudinal_accel,
                                             limits_.max_lateral_accel, limits_.max_curvature);

    PlannerDebugStats stats;
    // TRACKING은 아직 회피를 시작하지 않는 구간이다. 8초 충돌 lookahead가
    // 먼 정적 장애물까지 미리 중앙 경로를 전부 제거하면 stop/go가 발생하므로,
    // 잠금된 해당 장애물만 이 구간의 충돌 필터에서 제외한다. 다른
    // 장애물은 계속 필터링되고, 25m 진입 시 잠금 장애물도 즉시 다시 포함된다.
    std::vector<ObjectInfo> planning_obstacles = obstacles;
    if (avoidance_.active && avoidance_.phase == AvoidPhase::TRACKING && !begin_avoidance) {
        planning_obstacles.erase(
            std::remove_if(planning_obstacles.begin(), planning_obstacles.end(),
                           [this](const ObjectInfo& obj) {
                               return obj.id == avoidance_.obstacle_id;
                           }),
            planning_obstacles.end());
    }

    std::vector<FrenetPath> candidates;
    // AVOID의 고정 속도 대신 후보 곡률로 속도를 자동 결정한다.
    // 1차 일반 목표속도 생성 -> 최적 후보 곡률의 허용속도 계산 ->
    // 필요할 때만 낮은 목표속도로 2차 재생성 순서다.
    auto generate_adaptive_avoid = [&](PlannerCommand& avoid_cmd, double target_d,
                                       PlannerDebugStats& avoid_stats) {
        auto generate_once = [&](PlannerDebugStats& generated_stats) {
            generated_stats = PlannerDebugStats{};
            auto generated = ResolveManeuver(start, avoid_cmd, ref_, path_cfg_, limits_,
                                             lane_width_, planning_obstacles, vehicle_shape_,
                                             collision_cfg_, &generated_stats);
            EvaluateCosts(generated, cost_weights_, target_d);
            return generated;
        };

        std::vector<FrenetPath> generated = generate_once(avoid_stats);
        const FrenetPath* initial_best = SelectBestPath(generated);
        if (initial_best) {
            const double speed_limit = CandidateCurvatureSpeedLimit(
                *initial_best, ref_, limits_.max_lateral_accel);
            if (std::isfinite(speed_limit) && speed_limit + 0.1 < avoid_cmd.target_speed) {
                avoid_cmd.target_speed = speed_limit;
                generated = generate_once(avoid_stats);
            }
        }

        FilterByCartesianLateralAcceleration(generated, ref_, limits_.max_lateral_accel);
        // 추가 횡가속도 필터를 반영해 시각화 통계를 재계산한다.
        avoid_stats.combined_valid_after_curvature = 0;
        avoid_stats.combined_valid_after_collision = 0;
        for (const auto& path : generated) {
            if (path.valid || path.rejection_reason == RejectionReason::COLLISION) {
                ++avoid_stats.combined_valid_after_curvature;
            }
            if (path.valid) ++avoid_stats.combined_valid_after_collision;
        }
        return generated;
    };

    if (begin_avoidance) {
        // 장애물 d에서 자차/장애물 반폭과 SAT 안전 마진을 더한 지점을
        // 양쪽 목표로 삼는다. 기존 ±3m 절대값보다 장애물의 실제
        // 횡위치를 반영하므로 불필요하게 멀리 이동하지 않는다.
        constexpr double kDirectionExtraMargin = 0.3;
        const double clearance = 0.5 * (vehicle_shape_.width + avoidance_.obstacle_width)
                               + collision_cfg_.safety_margin + kDirectionExtraMargin;
        const double left_target = avoidance_.obstacle_d + clearance;
        const double right_target = avoidance_.obstacle_d - clearance;

        auto generate_side = [&](double target_d, PlannerDebugStats& side_stats,
                                 double& used_target_speed) {
            PlannerCommand side_cmd = cmd;
            side_cmd.avoidance_d_offset = target_d;
            auto side = generate_adaptive_avoid(side_cmd, target_d, side_stats);
            used_target_speed = side_cmd.target_speed;
            return side;
        };

        PlannerDebugStats left_stats, right_stats;
        double left_target_speed = cmd.target_speed;
        double right_target_speed = cmd.target_speed;
        std::vector<FrenetPath> left_candidates =
            generate_side(left_target, left_stats, left_target_speed);
        std::vector<FrenetPath> right_candidates =
            generate_side(right_target, right_stats, right_target_speed);
        const FrenetPath* left_best = SelectBestPath(left_candidates);
        const FrenetPath* right_best = SelectBestPath(right_candidates);

        bool choose_right = false;
        bool direction_found = left_best || right_best;
        if (!left_best && right_best) {
            choose_right = true;
        } else if (left_best && right_best) {
            const double tie_margin = std::max(1.0, std::fabs(left_best->cost_total)) * 0.05;
            choose_right = right_best->cost_total < left_best->cost_total ||
                (avoid_cfg_.prefer_right_when_equal &&
                 right_best->cost_total <= left_best->cost_total + tie_margin);
        }

        if (direction_found) {
            avoidance_.target_d = choose_right ? right_target : left_target;
            avoidance_.phase = AvoidPhase::SHIFT;
            cmd.avoidance_d_offset = avoidance_.target_d;
            cmd.target_speed = choose_right ? right_target_speed : left_target_speed;
            avoid_offset = avoidance_.target_d;
            candidates = choose_right ? std::move(right_candidates) : std::move(left_candidates);
            stats = choose_right ? right_stats : left_stats;
            std::printf("[FrenetPlanner] avoid direction locked: %s id=%d gap=%.2f "
                        "target_d=%.2f target_speed=%.2f valid(left=%zu right=%zu)\n",
                        choose_right ? "RIGHT" : "LEFT", avoidance_.obstacle_id,
                        avoidance_.obstacle_s - s, avoidance_.target_d, cmd.target_speed,
                        left_stats.combined_valid_after_collision,
                        right_stats.combined_valid_after_collision);
        } else {
            // 양쪽 모두 실패한 경우 방향을 잠그지 않고 두 집합을 모두
            // 시각화한다. 상위의 no-valid 처리가 안전 정지를 보낸 뒤
            // 다음 사이클에서 좌/우를 다시 평가한다.
            candidates = std::move(left_candidates);
            candidates.insert(candidates.end(),
                              std::make_move_iterator(right_candidates.begin()),
                              std::make_move_iterator(right_candidates.end()));
            stats.lateral_total = left_stats.lateral_total + right_stats.lateral_total;
            stats.lateral_valid = left_stats.lateral_valid + right_stats.lateral_valid;
            stats.longitudinal_total = left_stats.longitudinal_total + right_stats.longitudinal_total;
            stats.longitudinal_valid = left_stats.longitudinal_valid + right_stats.longitudinal_valid;
            stats.combined_total = left_stats.combined_total + right_stats.combined_total;
            stats.combined_valid_after_curvature = left_stats.combined_valid_after_curvature
                                                 + right_stats.combined_valid_after_curvature;
            stats.combined_valid_after_collision = 0;
            std::printf("[FrenetPlanner] avoid direction unavailable: id=%d gap=%.2f "
                        "valid(left=0 right=0)\n",
                        avoidance_.obstacle_id, avoidance_.obstacle_s - s);
        }
    } else {
        if (cmd.mode == AVOID) {
            candidates = generate_adaptive_avoid(cmd, cmd.avoidance_d_offset, stats);
        } else {
            candidates = ResolveManeuver(start, cmd, ref_, path_cfg_, limits_, lane_width_,
                                         planning_obstacles, vehicle_shape_, collision_cfg_, &stats);
            EvaluateCosts(candidates, cost_weights_, 0.0);
        }
    }

    if (cmd.mode != prev_mode_) {
        std::printf("[FrenetPlanner] mode: %s -> %s (s=%.2f d=%.3f)\n",
                    ModeName(prev_mode_), ModeName(cmd.mode), s, d);
        if (cmd.mode == AVOID) {
            std::printf("[FrenetPlanner]   avoid_id=%d phase=%s target_d=%.3f\n",
                        avoidance_.obstacle_id,
                        AvoidPhaseName(static_cast<int>(avoidance_.phase)), avoid_offset);
        } else if (cmd.mode == FOLLOWING) {
            const double gap = leader_s - s;
            const double desired_gap = following_cfg_.min_gap
                                     + following_cfg_.time_gap * leader_speed;
            std::printf("[FrenetPlanner]   leader_id=%d leader_s=%.2f leader_speed=%.2f "
                        "gap=%.2f desired_gap=%.2f\n",
                        following_.leader_id, leader_s, leader_speed, gap, desired_gap);
        }
        prev_mode_ = cmd.mode;
    }

    const FrenetPath* best = SelectBestPath(candidates);
    const int selected_index = best
        ? static_cast<int>(best - candidates.data())
        : -1;

    if (debug_writer_) {
        debug_writer_->Publish(ref_, ego, start, cmd.mode,
                               cmd.mode == AVOID
                                   ? AvoidPhaseName(static_cast<int>(avoidance_.phase)) : "NONE",
                               cmd.target_speed,
                               obstacles, candidates,
                               selected_index, stats);
    }

    if (!best) {
        std::printf("[FrenetPlanner] No valid candidate this cycle "
                    "(mode=%s, %zu generated)\n",
                    ModeName(cmd.mode), candidates.size());
        std::printf("[FrenetPlanner-DEBUG] ego: x=%.3f y=%.3f yaw=%.3f v=%.3f a=%.3f\n",
                     ego.x, ego.y, ego.yaw, ego.v, ego.a);
        std::printf("[FrenetPlanner-DEBUG] start: s=%.2f d=%.3f s_dot=%.2f s_ddot=%.2f d_dot=%.3f d_ddot=%.3f "
                     "target_speed=%.2f\n",
                     s, d, s_dot, s_ddot, d_dot, d_ddot, cmd.target_speed);
        std::printf("[FrenetPlanner-DEBUG] lateral %zu/%zu valid | longitudinal %zu/%zu valid | "
                     "combined %zu -> after curvature %zu -> after collision %zu valid\n",
                     stats.lateral_valid, stats.lateral_total,
                     stats.longitudinal_valid, stats.longitudinal_total,
                     stats.combined_total, stats.combined_valid_after_curvature,
                     stats.combined_valid_after_collision);
        return false;
    }

    // 모든 모드의 현재 상태를 20Hz 기준 약 4Hz로 출력한다. 모드 전환 로그만으로는
    // 후보가 사라지는 바로 그 순간이 LANE_KEEPING/AVOID 중 어느 상태였는지 알 수
    // 없으므로, s/d/속도와 필터 후 후보 수를 같은 줄에 남긴다.
    static int state_log_tick = 0;
    if (++state_log_tick % 5 == 0) {
        if (cmd.mode == FOLLOWING) {
            const double gap = leader_s - s;
            const double desired_gap = following_cfg_.min_gap
                                     + following_cfg_.time_gap * leader_speed;
            const double approach_speed = std::max(0.0, std::min(
                cmd.target_speed,
                leader_speed + following_cfg_.gap_gain * (gap - desired_gap)));
            std::printf("[FrenetPlanner-STATE] mode=FOLLOWING leader_id=%d s=%.2f d=%.3f "
                        "ego_speed=%.2f leader_speed=%.2f relative_speed=%.2f "
                        "gap=%.2f desired_gap=%.2f gap_error=%.2f approach_speed=%.2f chosen_d=%.3f "
                        "cost=%.2f candidates=%zu/%zu missing=%d\n",
                        following_.leader_id, s, d, ego.v, leader_speed,
                        ego.v - leader_speed, gap, desired_gap, gap - desired_gap,
                        approach_speed, best->d.empty() ? d : best->d.back(), best->cost_total,
                        stats.combined_valid_after_collision, stats.combined_total,
                        following_.missing_cycles);
        } else {
            std::printf("[FrenetPlanner-STATE] mode=%s phase=%s s=%.2f d=%.3f speed=%.2f target_speed=%.2f "
                        "target_d=%.3f chosen_d=%.3f cost=%.2f candidates=%zu/%zu\n",
                        ModeName(cmd.mode), cmd.mode == AVOID
                            ? AvoidPhaseName(static_cast<int>(avoidance_.phase)) : "NONE",
                        s, d, ego.v, cmd.target_speed,
                        cmd.mode == AVOID ? avoid_offset : 0.0,
                        best->d.empty() ? d : best->d.back(), best->cost_total,
                        stats.combined_valid_after_collision, stats.combined_total);
        }
    }

    out_path = ConvertToCartesianPath(*best, ref_);
    return true;
}
