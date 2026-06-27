#include "path_planner.hpp"
#include "../global/utils.hpp"

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

    auto sqDist = [&](int i) -> double {
        double dx = waypoints[i].x - ego_snap.x;
        double dy = waypoints[i].y - ego_snap.y;
        return dx*dx + dy*dy;
    };

    // ========================================
    // closest waypoint 탐색
    //   앞쪽으로만 탐색 + 헤딩 필터 (U턴 오탐 방지)
    // ========================================
    const int look_forward = 80;
    const int look_back    = 3;

    bool global_search = (std::sqrt(sqDist(idx)) > 15.0);
    int search_start = global_search ? 0 : std::max(0, idx - look_back);
    int search_end   = global_search ? n : std::min(n, idx + look_forward);

    int best = idx;
    double best_d2 = std::numeric_limits<double>::infinity();

    for (int i = search_start; i < search_end; ++i) {
        double wp_yaw = 0.0;
        if (i + 1 < n)
            wp_yaw = std::atan2(waypoints[i+1].y - waypoints[i].y,
                                waypoints[i+1].x - waypoints[i].x);
        else if (i > 0)
            wp_yaw = std::atan2(waypoints[i].y - waypoints[i-1].y,
                                waypoints[i].x - waypoints[i-1].x);

        double heading_dot = std::cos(ego_snap.yaw - wp_yaw);
        if (!global_search && heading_dot < -0.3) continue;

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

        double yaw_ref = 0.0;
        if (i + 1 < n)
            yaw_ref = std::atan2(waypoints[i+1].y - waypoints[i].y,
                                 waypoints[i+1].x - waypoints[i].x);
        else if (i > 0)
            yaw_ref = std::atan2(waypoints[i].y - waypoints[i-1].y,
                                 waypoints[i].x - waypoints[i-1].x);

        out_ref.yaw_ref.push_back(yaw_ref);
        out_ref.v_ref.push_back(velocityFromCurvature(waypoints[i].curvature));
    }

    // ========================================
    // Look-ahead 감속
    //   오버슈트 원인: 현재 waypoint 곡률만 보고 감속하면
    //   이미 커브에 진입한 후에야 감속 → 오버슈트 발생
    //
    //   해결: 앞쪽 look_ahead_count개 waypoint 중
    //   가장 낮은 v_ref를 현재 v_ref에 반영
    //   → 커브 진입 전에 미리 감속
    // ========================================
    const int look_ahead_count = 30;  // 약 3m 앞 (waypoint 간격 0.1m 기준)
    int sz = static_cast<int>(out_ref.v_ref.size());
    for (int i = 0; i < sz; ++i) {
        int end_j = std::min(sz, i + look_ahead_count);
        double min_v = out_ref.v_ref[i];
        for (int j = i + 1; j < end_j; ++j)
            min_v = std::min(min_v, out_ref.v_ref[j]);
        out_ref.v_ref[i] = min_v;
    }

    // v_ref smoothing: 계단식 속도 변화 완화
    if (out_ref.v_ref.size() >= 3) {
        std::vector<double> smoothed = out_ref.v_ref;
        int smooth_radius = 30;
        for (size_t i = 0; i < out_ref.v_ref.size(); ++i) {
            size_t s = (i > static_cast<size_t>(smooth_radius)) ? i - smooth_radius : 0;
            size_t e = std::min(out_ref.v_ref.size() - 1,
                                i + static_cast<size_t>(smooth_radius));
            double sum = 0.0; int cnt = 0;
            for (size_t j = s; j <= e; ++j) { sum += out_ref.v_ref[j]; ++cnt; }
            smoothed[i] = sum / cnt;
        }
        out_ref.v_ref = smoothed;
    }

    return !out_ref.empty();
}