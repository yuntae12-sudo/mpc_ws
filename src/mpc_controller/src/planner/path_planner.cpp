#include "path_planner.hpp"
#include "../global/utils.hpp"

// ========================================
// path.txt 기반 reference path 생성
//   mpc_node.cpp 에 있던 buildReferenceFromWaypoints() 를
//   Planner 로 분리한 것
//
//   나중에 Expert / PA / SA waypoint 5개로 교체할 때
//   이 함수만 교체하면 mpc_node.cpp 는 건드릴 필요 없음
// ========================================
bool buildReferenceFromWaypoints(
    const MPCState&              ego_snap,
    const std::vector<Waypoint>& waypoints,
    const MPCParams&             params,
    ReferencePath&               out_ref,
    int&                         closest_idx)
{
    out_ref.clear();
    if (waypoints.empty()) return false;

    int n = static_cast<int>(waypoints.size());
    int idx = clip(closest_idx, 0, n - 1);

    // 현재 캐시 인덱스까지의 거리 계산
    auto sqDist = [&](int i) -> double {
        double dx = waypoints[i].x - ego_snap.x;
        double dy = waypoints[i].y - ego_snap.y;
        return dx*dx + dy*dy;
    };

    // 캐시가 15m 이상 멀면 전체 탐색, 아니면 주변만 탐색
    bool global_search = (std::sqrt(sqDist(idx)) > 15.0);
    int start = global_search ? 0          : std::max(0, idx - 10);
    int end   = global_search ? n          : std::min(n, idx + 50);

    int best = idx;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (int i = start; i < end; ++i) {
        double d2 = sqDist(i);
        if (d2 < best_d2) { best_d2 = d2; best = i; }
    }
    closest_idx = best;

    // best 부터 ref_window 개 선택
    int last = std::min(n, best + params.ref_window);

    out_ref.x_ref.reserve(last - best);
    out_ref.y_ref.reserve(last - best);
    out_ref.yaw_ref.reserve(last - best);
    out_ref.v_ref.reserve(last - best);
    out_ref.k_ref.reserve(last - best);

    // 곡률 기반 target velocity 결정
    auto velocityFromCurvature = [&](double k) -> double {
        if (k > params.curve_th_sharp) return params.curve_vel_sharp;
        if (k > params.curve_th_mid)   return params.curve_vel_mid;
        if (k > params.curve_th_mild)  return params.curve_vel_mild;
        return params.target_vel;
    };

    for (int i = best; i < last; ++i) {
        out_ref.x_ref.push_back(waypoints[i].x);
        out_ref.y_ref.push_back(waypoints[i].y);
        out_ref.k_ref.push_back(waypoints[i].curvature);

        // yaw: 인접 점의 방향
        double yaw_ref = 0.0;
        if (i + 1 < n) {
            yaw_ref = std::atan2(waypoints[i+1].y - waypoints[i].y,
                                 waypoints[i+1].x - waypoints[i].x);
        } else if (i > 0) {
            yaw_ref = std::atan2(waypoints[i].y - waypoints[i-1].y,
                                 waypoints[i].x - waypoints[i-1].x);
        }
        out_ref.yaw_ref.push_back(yaw_ref);
        out_ref.v_ref.push_back(velocityFromCurvature(waypoints[i].curvature));
    }

    // v_ref smoothing: 계단식 속도 변화 완화
    if (out_ref.v_ref.size() >= 3) {
        std::vector<double> smoothed = out_ref.v_ref;
        int smooth_radius = 30;
        for (size_t i = 0; i < out_ref.v_ref.size(); ++i) {
            size_t s = (i > static_cast<size_t>(smooth_radius)) ? i - smooth_radius : 0;
            size_t e = std::min(out_ref.v_ref.size() - 1,
                                i + static_cast<size_t>(smooth_radius));
            double sum = 0.0;
            int cnt = 0;
            for (size_t j = s; j <= e; ++j) { sum += out_ref.v_ref[j]; ++cnt; }
            smoothed[i] = sum / cnt;
        }
        out_ref.v_ref = smoothed;
    }

    return !out_ref.empty();
}