#pragma once

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>

#include "frenet_planner/frenet/ref_line.hpp"
#include "frenet_planner/frenet/path_generator.hpp"
#include "frenet_planner/frenet/cost.hpp"
#include "frenet_planner/frenet/avoidance_selector.hpp"
#include "frenet_planner/frenet/collision_checker.hpp"
#include "frenet_planner/frenet/leader_selector.hpp"
#include "frenet_planner/frenet/merge_selector.hpp"
#include "frenet_planner/global/data_logger.hpp"
#include "frenet_planner/global/global.hpp"
#include "frenet_planner/math/frenet_converter.hpp"
#include "frenet_planner/visualization/planner_debug_writer.hpp"

// Frenet Frame Path Planner: 전역 경로(RefLine)를 기준으로 한 Local Planner.
// FSM(behavior_planner)이 아직 연동되지 않아서, 모드 전환 조건을 지금은
// Plan() 안에 직접 하드코딩해뒀다(내 차선을 막는 정지/저속 장애물이 있으면
// AVOID, 그 다음 대상 차선에 sa/sb(앞/뒤 차량)가 모두 있으면 MERGE, 그
// 다음 선두 차량이 있으면 FOLLOWING, 다 없으면 LANE_KEEPING). MERGE는
// 설정된 회전교차로 conflict_s 접근구간에서 시간 gap을 판단한다. behavior_planner
// 연동 시 이 조건문들을 걷어내고 FSM이 준 mode를 그대로 쓰도록 교체한다.
class FrenetPlanner {
public:
    enum class PlanFailureReason {
        NONE,
        COLLISION_BLOCKED,
        TRANSIENT_GENERATION_FAILURE,
        NOT_INITIALIZED,
    };

    // yaml_path: frenet_planner/config/params.yaml
    bool Init(const std::string& yaml_path);

    // ego: 현재 차량 상태(Cartesian), obstacles: 장애물 목록.
    // 반환: 성공 시 true + out_path에 결과. 유효 후보가 없으면 false.
    bool Plan(const CartesianState& ego, const std::vector<ObjectInfo>& obstacles,
              CartesianPath& out_path);

    // Plan()이 반환하는 CartesianPath의 샘플 간격 [s] (params.yaml의 planner.dt).
    // MPC 쪽에서 이 값을 알아야 시간 기준으로 재샘플링해 궤적 추종(time-aligned
    // tracking)을 할 수 있다 (frenet_planner.cpp의 dt와 mpc_params.dt가 서로
    // 달라서 그냥 index를 맞춰 쓸 수 없음).
    double sample_dt() const { return path_cfg_.dt; }

    // 진단용: 매 Plan() 호출마다 갱신되는 현재 d(차선 중앙 기준 오프셋)/d_dot(횡방향
    // 속도). 횡방향 오차가 조향 포화 때문인지(모델/액추에이션 문제) 아니면 여유가
    // 있는데도 못 줄이는지(모델 불일치) 구분하려고 main.cpp 로그에서 steer_norm과
    // 같이 찍어본다.
    double last_d() const { return last_d_; }
    double last_d_dot() const { return last_d_dot_; }
    PlanFailureReason last_failure_reason() const { return last_failure_reason_; }

private:
    // TRACKING은 외부 AVOID 모드가 아니다. 정적 장애물 ID/방향만
    // 미리 잠그고 주행은 LANE_KEEPING을 계속하는 내부 상태다.
    enum class AvoidPhase { TRACKING, SHIFT, PASS };

    struct AvoidanceContext {
        bool active = false;
        int obstacle_id = -1;
        AvoidPhase phase = AvoidPhase::TRACKING;
        double obstacle_s = 0.0;
        double obstacle_d = 0.0;
        double obstacle_width = 0.0;
        double obstacle_length = 0.0;
        double target_d = 0.0;
    };

    struct FollowingContext {
        bool active = false;
        int leader_id = -1;
        double leader_s = 0.0;
        double leader_d = 0.0;
        double leader_speed = 0.0;
        double leader_accel = 0.0;
        int missing_cycles = 0;
    };

    enum class MergePhase { APPROACH, WAIT, COMMIT, CROSS, CLEAR };

    struct MergeContext {
        bool active = false;
        MergeType type = MergeType::NONE;
        int highway_zone_index = -1;
        int highway_conflict_index = 0;
        MergePhase phase = MergePhase::APPROACH;
        bool gap_safe = false;
        bool gap_locked = false;
        bool commit_pending = false;  // 실제 후보 검증 전의 임시 COMMIT 요청
        int safe_cycles = 0;
        double entry_time = 0.0;
        double clear_time = 0.0;
        int preceding_id = -1;
        double preceding_time = -1.0;
        int following_id = -1;
        double following_time = -1.0;
        size_t crossing_vehicle_count = 0;
        CartesianPath locked_path;
        double locked_elapsed = 0.0;
        size_t locked_path_index = 0;  // wall-clock이 아니라 Ego의 실제 공간 진행 위치
    };

    struct ObjectTrack {
        double heading = 0.0;
        double yaw_rate = 0.0;
        int missing_cycles = 0;
    };

    // 저속(ego.v < kLowSpeedFallbackV) 전용 fallback: CartesianToFrenet/FrenetToCartesian의
    // d_prime = d_dot/s_dot 계산은 s_dot(≈속도)로 나누는 구조라 저속에서 수학적으로
    // 특이점을 가진다("고속 모드 전용" - frenet_converter.cpp 주석 참고). 그 특이점 있는
    // 경계조건으로 만든 quintic 후보가 정지 근처를 지나 실제로 움직이기 시작하는
    // 시점에 억지로 꺾인 모양이 되어 곡률필터에 걸리는 문제가 실측으로 확인됐다.
    // 저속에서는 이 경계조건 역산 자체를 안 쓰고, s/d를 직접(시간에 따라 완만하게)
    // 설계해 ComputeGeometricPath로만 렌더링한다 - 나누기가 전혀 없어 특이점이 없다.
    // 활성 AVOID가 있으면 중앙이 아니라 저장된 회피 target_d를 유지/추종한다.
    bool PlanLowSpeedFallback(const CartesianState& ego, double s, double d, double target_d,
                              CartesianPath& out_path, double duration_s = 2.0) const;

    // 일반 저속 fallback의 크롤링 가속 프로파일 상수. 회전교차로 MERGE는
    // 이 고정 2 m/s 프로파일을 쓰지 않고 정상 MERGE 후보 생성기로 처리한다.
    static constexpr double kCrawlAccel = 0.5;       // [m/s^2] 완만한 가속
    static constexpr double kMinCreepSpeed = 0.3;    // [m/s] 최소 전진속도 (0 근처에 갇히지 않도록)
    static constexpr double kCrawlSpeedCap = 2.0;    // [m/s] 이 fallback 안에서의 속도 상한

    RefLine ref_;
    PathGeneratorConfig path_cfg_{};
    KinematicLimits limits_{};
    CostWeights cost_weights_{};
    VehicleShape vehicle_shape_{};
    CollisionCheckConfig collision_cfg_{};
    CurveSpeedConfig curve_speed_cfg_{};
    FollowingConfig following_cfg_{};
    AvoidConfig avoid_cfg_{};
    MergeConfig merge_cfg_{};
    HighwayMergeConfig highway_merge_cfg_{};
    PlannerVisualizationConfig visualization_cfg_{};
    std::unique_ptr<PlannerDebugWriter> debug_writer_;
    double wheelbase_ = 3.0;
    double lane_width_ = 3.5;
    bool ref_loaded_ = false;

    // 직전 사이클의 s (FindClosestS 탐색범위 힌트용 - ref_line.hpp 주석 참고,
    // 트랙이 자기 자신과 가까운 구간에서 최근접점이 엉뚱한 s로 튀는 문제 방지).
    double last_s_ = 0.0;
    bool has_last_s_ = false;
    double last_d_ = 0.0;
    double last_d_dot_ = 0.0;
    PlanFailureReason last_failure_reason_ = PlanFailureReason::NONE;

    // 모드 전환(LANE_KEEPING/FOLLOWING/AVOID) 시점만 로그로 찍기 위한 이전 사이클 값.
    BehaviorState prev_mode_ = LANE_KEEPING;
    MergePhase prev_merge_phase_ = MergePhase::CLEAR;
    AvoidanceContext avoidance_{};
    FollowingContext following_{};
    MergeContext merge_{};
    std::unordered_map<int, ObjectTrack> object_tracks_{};
};
