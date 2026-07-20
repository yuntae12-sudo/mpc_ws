#pragma once

#include <string>
#include <vector>

#include "frenet_planner/frenet/ref_line.hpp"
#include "frenet_planner/frenet/path_generator.hpp"
#include "frenet_planner/frenet/cost.hpp"
#include "frenet_planner/frenet/collision_checker.hpp"
#include "frenet_planner/global/data_logger.hpp"
#include "frenet_planner/global/global.hpp"
#include "frenet_planner/math/frenet_converter.hpp"

// Frenet Frame Path Planner: 전역 경로(RefLine)를 기준으로 한 Local Planner.
// FSM이 아직 없어서 지금은 LANE_KEEPING 고정 + 곡률 기반 사전 감속만 자체 반영한다.
// (behavior_planner 연동은 FSM 완성 후 별도 단계에서 붙일 예정)
class FrenetPlanner {
public:
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

private:
    RefLine ref_;
    PathGeneratorConfig path_cfg_{};
    KinematicLimits limits_{};
    CostWeights cost_weights_{};
    VehicleShape vehicle_shape_{};
    CollisionCheckConfig collision_cfg_{};
    CurveSpeedConfig curve_speed_cfg_{};
    double wheelbase_ = 3.0;
    double lane_width_ = 3.5;
    bool ref_loaded_ = false;

    // 직전 사이클의 s (FindClosestS 탐색범위 힌트용 - ref_line.hpp 주석 참고,
    // 트랙이 자기 자신과 가까운 구간에서 최근접점이 엉뚱한 s로 튀는 문제 방지).
    double last_s_ = 0.0;
    bool has_last_s_ = false;
};
