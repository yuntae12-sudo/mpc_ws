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
                LeaderTarget& out_leader) {
    const double half_lane = lane_width * 0.5;

    bool found = false;
    double best_gap = std::numeric_limits<double>::max();
    int best_id = -1;
    double best_s = 0.0, best_d = 0.0, best_s_dot = 0.0;
    const double track_length = ref.points.empty() ? 0.0 : ref.points.back().s;

    for (const auto& obj : obstacles) {
        double s, d, s_dot;
        ProjectObjectToFrenet(ref, obj, s, d, s_dot);

        // 정지/저속 객체는 FOLLOWING 대상이 아니다. 이전에는 속도 조건이 없어
        // 정적 장애물이 60m 탐색범위에 들어오는 즉시 FOLLOWING으로 분류됐고,
        // 먼 정지점까지 짧은 horizon 안에 도달해 완전히 멈추는 quintic 후보가
        // 종방향 가속도 제한을 전부 위반하면서 후보 0개 -> 정지로 이어졌다.
        // 경로 방향으로 투영한 속도를 사용해 횡단 객체도 동일 기준으로 분류한다.
        // FOLLOWING은 참조선과 같은 방향으로 진행하는 차량만 대상이다.
        // abs()를 쓰면 s_dot<0인 역주행/대항 차량도 leader로 선택된다.
        if (s_dot <= cfg.min_leader_speed) continue;

        double gap = s - ego.s;
        if (gap <= 0.0 && track_length > 0.0) gap += track_length;
        if (gap <= 0.0) continue;                          // 뒤에 있는 차량은 제외
        if (gap > cfg.max_leader_search_s) continue;        // 탐색 범위 밖
        if (std::fabs(d) > half_lane) continue;             // 내 차선 밖

        if (gap < best_gap) {
            best_gap = gap;
            best_id = obj.id;
            best_s = ego.s + gap;  // 폐루프 seam 넘어서도 ego 앞의 연속 s로 전달
            best_d = d;
            best_s_dot = s_dot;
            found = true;
        }
    }

    if (!found) return false;

    out_leader.id = best_id;
    out_leader.s = best_s;
    out_leader.d = best_d;
    out_leader.speed = best_s_dot;
    out_leader.accel = 0.0;
    return true;
}
