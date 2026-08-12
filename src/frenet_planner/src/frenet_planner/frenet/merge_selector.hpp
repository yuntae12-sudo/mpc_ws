#ifndef FRENET_MERGE_SELECTOR_HPP
#define FRENET_MERGE_SELECTOR_HPP

#include <vector>
#include <unordered_map>

#include "frenet_planner/frenet/ref_line.hpp"
#include "frenet_planner/frenet/collision_checker.hpp"
#include "frenet_planner/global/data_logger.hpp"
#include "frenet_planner/global/global.hpp"

struct RoundaboutGap {
    bool safe = false;
    bool confirmed = false;
    double entry_time = 0.0;
    double clear_time = 0.0;
    int preceding_id = -1;
    double preceding_time = -1.0;
    int following_id = -1;
    double following_time = -1.0;
    size_t crossing_vehicle_count = 0;
};

// 순환 차량을 등속 직선 예측했을 때 conflict point에 가장 가까워지는 시간을
// 구하고, Ego가 들어갈 수 있는 첫 시간 gap을 반환한다. 횡경로는 global path가
// 담당하며 이 함수는 회전교차로 진입 타이밍만 판단한다.
RoundaboutGap FindRoundaboutGap(const RefLine& ref, const FrenetState& ego,
                                const std::vector<ObjectInfo>& obstacles,
                                const MergeConfig& cfg, double desired_speed,
                                double max_ego_accel,
                                const std::unordered_map<int, double>* yaw_rates = nullptr);

// Global Path가 램프에서 본선으로 이어지는 고주로 합류용. 목표 corridor에 투영된
// 차량의 합류 완료시점 앞/뒤 거리와 TTC를 함께 만족하는 첫 진입시간을 찾는다.
RoundaboutGap FindHighwayMergeGap(const RefLine& ref, const FrenetState& ego,
                                  const std::vector<ObjectInfo>& obstacles,
                                  const HighwayMergeConfig& cfg,
                                  const HighwayMergeCheckpoint& checkpoint,
                                  const VehicleShape& ego_shape,
                                  const CollisionCheckConfig& collision_cfg,
                                  double desired_speed, double max_ego_accel);

#endif
