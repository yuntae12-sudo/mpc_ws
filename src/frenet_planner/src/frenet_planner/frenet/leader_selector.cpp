#include "frenet_planner/frenet/leader_selector.hpp"

#include <cmath>
#include <limits>

#include "frenet_planner/math/frenet_converter.hpp"

void ProjectObjectToFrenet(const RefLine& ref, const ObjectInfo& obj,
                           double& out_s, double& out_d, double& out_s_dot) {
    CartesianState cs{};
    cs.x = obj.x;
    cs.y = obj.y;
    cs.yaw = obj.heading;
    cs.kappa = 0.0;   // CartesianToFrenet의 s_dot/d 계산에는 안 쓰임(FrenetToCartesian 역방향 전용 필드)
    cs.v = obj.speed;
    cs.a = 0.0;       // 가속도 정보 없음 - s_ddot은 안 쓰므로 무관

    double s, s_dot, s_ddot, d, d_prime, d_pprime;
    CartesianToFrenet(ref, cs, s, s_dot, s_ddot, d, d_prime, d_pprime, nullptr);

    out_s = s;
    out_d = d;
    out_s_dot = s_dot;
}

bool FindLeader(const RefLine& ref, const FrenetState& ego,
                const std::vector<ObjectInfo>& obstacles,
                double lane_width, const FollowingConfig& cfg,
                double& out_leader_s, double& out_leader_speed, double& out_leader_accel) {
    const double half_lane = lane_width * 0.5;

    bool found = false;
    double best_gap = std::numeric_limits<double>::max();
    double best_s = 0.0, best_s_dot = 0.0;

    for (const auto& obj : obstacles) {
        double s, d, s_dot;
        ProjectObjectToFrenet(ref, obj, s, d, s_dot);

        const double gap = s - ego.s;
        if (gap <= 0.0) continue;                          // 뒤에 있는 차량은 제외
        if (gap > cfg.max_leader_search_s) continue;        // 탐색 범위 밖
        if (std::fabs(d) > half_lane) continue;             // 내 차선 밖

        if (gap < best_gap) {
            best_gap = gap;
            best_s = s;
            best_s_dot = s_dot;
            found = true;
        }
    }

    if (!found) return false;

    out_leader_s = best_s;
    out_leader_speed = best_s_dot;
    out_leader_accel = 0.0;
    return true;
}
