#ifndef FRENET_PATH_GENERATOR_HPP
#define FRENET_PATH_GENERATOR_HPP

#include "frenet_planner/frenet/collision_checker.hpp"
#include "frenet_planner/frenet/ref_line.hpp"
#include "frenet_planner/global/global.hpp"

// =========================================================
// 후보 궤적 샘플링 격자(grid) 설정
//
// 논문 Sec.IV-A / V-A / V-B 는 모두 "여러 종료조건(di 또는 Δsi 또는 Δṡi)과
// 여러 종료시간 Tj 의 조합"으로 후보 집합을 만든다. 이 프로젝트에서는
// lateral과 longitudinal이 Sec.VI에서 "Tlat × Tlon 모든 조합"으로 묶이는데,
// 이때 서로 다른 T를 가진 후보끼리 묶으면 두 다항식의 정의 구간이 달라져
// 궤적을 만들 수 없다. 그래서 lateral/longitudinal이 동일한 T_j 격자를
// 공유하도록 하고(time_horizon 하나로 통일), CombineLateralLongitudinal은
// 같은 T를 가진 후보끼리만 묶는다.
//
// (min, max, step)으로 표현: 실제 후보값은 min부터 max까지 step 간격
// (parameter.yaml에서 로드될 예정, 지금은 코드에서 직접 채워도 됨)
// =========================================================

struct SamplingRange {
    double min;
    double max;
    double step;
};

struct PathGeneratorConfig {
    SamplingRange lateral_d1;      // 목표 횡방향 오프셋 d1 후보 범위 [m] (Sec.IV-A)
    double lateral_target_tolerance = 0.5; // [m] 모드가 지정한 목표 d 주변 탐색 폭
    SamplingRange time_horizon;    // 종료시간 Tj 후보 범위 [s] (lateral/longitudinal 공통)
    SamplingRange delta_s;         // following/merging/stopping용 Δsi 범위 [m] (Sec.V-A)
    SamplingRange delta_s_dot;     // velocity keeping용 Δṡi 범위 [m/s] (Sec.V-B)
    double dt;                     // 최종 궤적을 샘플링할 시간 간격 [s]
};

// =========================================================
// 킨매틱 실현가능성 한계값
// 논문 Sec.VI 1번째 문단: "각 집합은 결합 전에 outsized 가속도(s̈, d̈/d″)
// 값에 대해 먼저 체크된다" — lateral/longitudinal 집합을 합치기 *전에*
// 각자 필터링. 곡률(κx) 체크는 결합 이후 Cartesian 변환을 해야 가능하므로
// (frenet_converter 필요) 별도 단계로 분리.
// =========================================================

struct KinematicLimits {
    double max_lateral_accel;      // |d̈| 허용치 [m/s^2]
    double max_longitudinal_accel; // |s̈| 허용치 [m/s^2]
    double max_curvature;          // |kappa_x| 허용치 [1/m] (차량 최소 회전반경 역수)
};

// =========================================================
// [Sec.IV-A] 횡방향(lateral) 후보 집합 생성 — 고속 모드 전용 (d(t) quintic)
//
// D0 = [d0, ḋ0, d̈0] = start (FrenetState의 d, d_d, d_dd)
// 종료조건 [d1, 0, 0, Tj] 를 cfg.lateral_d1 × cfg.time_horizon 격자로 순회하며
// MakeQuintic으로 각 후보를 만들고, cfg.dt 간격으로 샘플링해 FrenetPath.d/d_d/d_dd,
// t 를 채운다. (FrenetPath.s 계열은 비워둔 채로 반환 — longitudinal과 결합 시 채워짐)
//
// TODO(추후 개발 필요, FSM 저속 처리와 함께): Sec.IV-B의 저속 모드
// (d(s) quintic, 비홀로노믹 곡률 제약)는 아직 미구현. 지금은 항상 고속
// (d(t)) 모드만 생성한다.
//
// 저속(|ṡ0| < kLowSpeedThreshold) 임시 처리: d(t) quintic은 시간 기준이라
// 정지 근처에서는 s가 거의 안 움직이는데도 d1로 정해진 시간 T 안에 무조건
// 도달하려 해서 옆으로 미끄러지는 것과 같은 궤적이 나온다(예: s_dot≈0,
// s_ddot=0.3 상태에서 t=0.2s 동안 실제 전진량은 6mm뿐인데, d1=-3.0 목표
// 후보의 d_dot은 이미 -0.35 m/s에 달함 — 곡률 필터가 정확히 이 비현실적
// 후보들을 걸러내면서 저속 시작 시 후보가 전부 무효화되는 문제로 이어짐).
// Sec.IV-B를 구현하기 전까지는, 저속 구간에서 d1을 start.d 하나로 고정해
// (도착조건과 시작조건이 사실상 같아 d_dot이 거의 0에 머무름) 최소한 직진
// 가속은 가능하게 하고, 정상 속도가 되면 원래의 d1 격자 탐색을 재개한다.
// =========================================================

std::vector<FrenetPath> GenerateLateralCandidates(const FrenetState& start,
                                                   const PathGeneratorConfig& cfg);

// =========================================================
// [Sec.V-A] Following — 선두 차량과의 constant-time-gap 추종
//
// 선두 차량이 등가속(leader_accel=const)이라 가정하고 시간 적분한
// s_lv(t), ṡ_lv(t) 로부터
//   s_target(t)  = s_lv(t) - [min_gap + time_gap * ṡ_lv(t)]
//   ṡ_target(t)  = ṡ_lv(t) - time_gap * leader_accel
//   s̈_target(t)  = leader_accel                              (논문 Sec.V-A "Following")
// 를 구성한다. 목표가 현재 horizon 밖에 있을 때는
//   v_approach = clamp(v_lv + gap_gain * (gap - desired_gap), 0, cruise_target_speed)
// 로 접근하며, 목표가 horizon 안에 들어오면 위 constant-time-gap 종료조건을 쓴다.
// 각 Tj에서 [s_target(Tj)+Δsi, ṡ_target(Tj), s̈_target(Tj), Tj]를 종료조건으로 하는
// quintic 후보 집합을 cfg.delta_s × cfg.time_horizon 격자로 생성한다.
// =========================================================

std::vector<FrenetPath> GenerateFollowingCandidates(const FrenetState& start,
                                                     double leader_s,
                                                     double leader_speed,
                                                     double leader_accel,
                                                     double time_gap,
                                                     double min_gap,
                                                     double cruise_target_speed,
                                                     double gap_gain,
                                                     const PathGeneratorConfig& cfg);

// =========================================================
// [Sec.V-A] Stopping — 정지선/정지점 s_stop에서 정지
//
// s_target ≡ s_stop, ṡ_target ≡ 0, s̈_target ≡ 0 (논문 명시)
// Following과 동일하게 Δsi × Tj 격자로 quintic 후보 집합 생성.
// =========================================================

std::vector<FrenetPath> GenerateStoppingCandidates(const FrenetState& start,
                                                    double stop_s,
                                                    const PathGeneratorConfig& cfg);

// =========================================================
// [Sec.V-A] Merging — 두 차량 sa(t), sb(t) 사이로 병합
//
// s_target(t) = 1/2 * [sa(t) + sb(t)]  (논문 식 (3))
// sa, sb 모두 leader와 동일하게 등가속 예측을 적용해 시간적분한 뒤 평균낸다.
// =========================================================

std::vector<FrenetPath> GenerateMergingCandidates(const FrenetState& start,
                                                   double sa, double sa_speed, double sa_accel,
                                                   double sb, double sb_speed, double sb_accel,
                                                   const PathGeneratorConfig& cfg);

// 회전교차로 conflict point를 지정 시간에 통과하도록 위치/속도를 함께 고정한다.
std::vector<FrenetPath> GenerateMergeArrivalCandidates(const FrenetState& start,
                                                        double conflict_s,
                                                        double entry_time,
                                                        double entry_speed,
                                                        const PathGeneratorConfig& cfg);

// =========================================================
// [Sec.V-B] Velocity Keeping — 목표 속도 유지 (quartic, 종료 위치 자유)
//
// 종료조건 [ṡd + Δṡi, 0, Tj] 를 cfg.delta_s_dot × cfg.time_horizon 격자로
// 순회하며 MakeQuartic으로 후보를 만든다.
// =========================================================

std::vector<FrenetPath> GenerateVelocityKeepingCandidates(const FrenetState& start,
                                                           double target_speed,
                                                           const PathGeneratorConfig& cfg);

// =========================================================
// [Sec.VI 1단] 결합 전 사전 필터링
// 각 집합 안에서 |d̈|(또는 |s̈|)가 limits를 넘는 후보를 valid=false 처리.
// (논문 그림의 회색/검정 구분과 동일한 목적 — 지우지 않고 valid 플래그만 갱신해
//  디버그 시각화에 그대로 쓸 수 있게 한다)
// =========================================================

void FilterLateralByAcceleration(std::vector<FrenetPath>& candidates,
                                  const KinematicLimits& limits);

void FilterLongitudinalByAcceleration(std::vector<FrenetPath>& candidates,
                                       const KinematicLimits& limits);

// =========================================================
// [Sec.VI 2단] Tlat × Tlon 조합
//
// 동일한 T(종료시간)를 가진 lateral/longitudinal 후보끼리만 묶어
// 완전한 FrenetPath(t, d.., s..)를 만든다. (헤더 상단 설계 노트 참고)
// valid=false인 후보는 건너뛴다.
// =========================================================

std::vector<FrenetPath> CombineLateralLongitudinal(const std::vector<FrenetPath>& lateral_set,
                                                    const std::vector<FrenetPath>& longitudinal_set);

// =========================================================
// [Sec.VI 3단] 결합 이후 곡률(kappa_x) 필터링
// 각 샘플을 frenet_converter::FrenetToCartesian으로 변환해 kappa_x를 구하고
// limits.max_curvature를 넘으면 valid=false 처리.
// =========================================================

void FilterByCurvature(std::vector<FrenetPath>& combined,
                        const RefLine& ref,
                        const KinematicLimits& limits);

// =========================================================
// [Sec.VII] resolveManeuver — FSM ↔ Path Generator 연동 지점
//
// PlannerCommand.mode에 따라 활성화할 longitudinal 모드를 결정하고
// (예: LANE_KEEPING -> VelocityKeeping, FOLLOWING -> Following,
//      STOP/INTERSECTION_WAIT -> Stopping, LANE_CHANGE_* -> VelocityKeeping
//      + lateral target_lane 반영 등), lateral 후보 집합과 해당 longitudinal
// 후보 집합(들)을 생성해 Tlat × Tlon으로 결합한 뒤, 사전/사후 필터링까지
// 적용한 전체 후보 목록을 반환한다.
//
// 비용 계산(C_lat, C_lon)과 충돌 체크, 그리고 여러 활성 모드 중 최종 1개를
// 고르는 override 로직(Sec.VII, "가장 작은 초기 저크값" 기준)은 이 함수의
// 책임이 아니라 이후 cost.hpp / collision_checker.hpp가 담당한다.
// (path_generator는 "후보를 만드는" 역할까지만)
// =========================================================

// 실패 원인 진단용(정상 경로에는 영향 없음, nullptr이면 아무것도 안 함):
// 어느 필터 단계에서 후보가 다 죽었는지 구분하기 위한 카운터.
struct PlannerDebugStats {
    size_t lateral_total = 0, lateral_valid = 0;
    size_t longitudinal_total = 0, longitudinal_valid = 0;
    size_t combined_total = 0, combined_valid_after_curvature = 0;
    size_t combined_valid_after_collision = 0;
};

// obstacles/vehicle_shape/collision_cfg: Sec.VI 충돌 필터(FilterByCollision,
// SAT 기반)를 곡률 필터 다음에 적용한다. AVOID 전용이 아니라 모든 모드에
// 공통으로 적용된다(논문 Sec.VI도 충돌검사를 모드와 무관하게 항상 수행) -
// 장애물이 없거나 멀면 아무 후보도 걸러지지 않아 기존 동작과 동일하다.
// obstacles가 비어있으면(장애물 없음) 호출 자체를 건너뛴다.
std::vector<FrenetPath> ResolveManeuver(const FrenetState& start,
                                         const PlannerCommand& cmd,
                                         const RefLine& ref,
                                         const PathGeneratorConfig& cfg,
                                         const KinematicLimits& limits,
                                         double lane_width,
                                         const std::vector<ObjectInfo>& obstacles,
                                         const VehicleShape& vehicle_shape,
                                         const CollisionCheckConfig& collision_cfg,
                                         PlannerDebugStats* stats = nullptr);

#endif
