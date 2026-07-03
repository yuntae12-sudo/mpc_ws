#include "frenet/ref_line.hpp"
#include <algorithm>

namespace {

// theta는 atan2 결과라 ±π 경계에서 불연속(wrap-around)이 생김.
// 중앙/전후방 차분으로 kappa(=d_theta/ds)를 구할 때 이 불연속을 보정하지 않으면
// 곡선이 ±π를 지나는 순간 kappa가 튀는 버그가 생긴다.
double NormalizeAngleDiff(double dtheta) {
    while (dtheta > M_PI)  dtheta -= 2.0 * M_PI;
    while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
    return dtheta;
}

}  // namespace

// =========================================================
// BuildRefLine
// =========================================================

RefLine BuildRefLine(const std::vector<double>& wx, const std::vector<double>& wy) {

    if (wx.size() < 2 || wx.size() != wy.size())
        throw std::invalid_argument("Waypoints must have at least 2 points and matching sizes");

    RefLine ref;
    int n = static_cast<int>(wx.size());
    ref.points.resize(n);

    // 1. arc length s 누적 계산
    ref.points[0].x = wx[0];
    ref.points[0].y = wy[0];
    ref.points[0].s = 0.0;

    for (int i = 1; i < n; i++) {
        double dx = wx[i] - wx[i - 1];
        double dy = wy[i] - wy[i - 1];

        ref.points[i].x = wx[i];
        ref.points[i].y = wy[i];
        ref.points[i].s = ref.points[i - 1].s + std::sqrt(dx * dx + dy * dy);
    }

    // 2. theta: 중앙 차분 (양 끝은 전방/후방 차분)
    for (int i = 0; i < n; i++) {
        double dx, dy;
        if (i == 0) {
            dx = wx[1] - wx[0];
            dy = wy[1] - wy[0];
        } else if (i == n - 1) {
            dx = wx[n - 1] - wx[n - 2];
            dy = wy[n - 1] - wy[n - 2];
        } else {
            dx = wx[i + 1] - wx[i - 1];
            dy = wy[i + 1] - wy[i - 1];
        }
        ref.points[i].theta = std::atan2(dy, dx);
    }

    // 3. kappa: d_theta/ds (중앙 차분, wrap-around 보정 포함)
    for (int i = 0; i < n; i++) {
        if (i == 0) {
            double d_theta = NormalizeAngleDiff(ref.points[1].theta - ref.points[0].theta);
            double ds = ref.points[1].s - ref.points[0].s;

            ref.points[0].kappa = (ds > 1e-9) ? d_theta / ds : 0.0;

        } else if (i == n - 1) {
            double d_theta = NormalizeAngleDiff(ref.points[n - 1].theta - ref.points[n - 2].theta);
            double ds = ref.points[n - 1].s - ref.points[n - 2].s;

            ref.points[n - 1].kappa = (ds > 1e-9) ? d_theta / ds : 0.0;

        } else {
            double d_theta = NormalizeAngleDiff(ref.points[i + 1].theta - ref.points[i - 1].theta);
            double ds = ref.points[i + 1].s - ref.points[i - 1].s;

            ref.points[i].kappa = (ds > 1e-9) ? d_theta / ds : 0.0;
        }
    }

    // 4. d_kappa: d_kappa/ds (중앙 차분)
    for (int i = 0; i < n; i++) {
        if (i == 0) {
            double dk = ref.points[1].kappa - ref.points[0].kappa;
            double ds = ref.points[1].s - ref.points[0].s;

            ref.points[0].d_kappa = (ds > 1e-9) ? dk / ds : 0.0;

        } else if (i == n - 1) {
            double dk = ref.points[n - 1].kappa - ref.points[n - 2].kappa;
            double ds = ref.points[n - 1].s - ref.points[n - 2].s;

            ref.points[n - 1].d_kappa = (ds > 1e-9) ? dk / ds : 0.0;

        } else {
            double dk = ref.points[i + 1].kappa - ref.points[i - 1].kappa;
            double ds = ref.points[i + 1].s - ref.points[i - 1].s;

            ref.points[i].d_kappa = (ds > 1e-9) ? dk / ds : 0.0;
        }
    }

    return ref;
}

// =========================================================
// Interpolate — 선형 보간
// FindClosestS가 각 Newton 반복마다 임의의 실수 s에서 RefPoint를
// 필요로 하기 때문에, Newton 함수보다 먼저 정의되어야 한다.
// =========================================================

RefPoint Interpolate(const RefLine& ref, double s) {
    const auto& pts = ref.points;
    int n = static_cast<int>(pts.size());

    // 범위 클램핑
    if (s <= pts.front().s) return pts.front();
    if (s >= pts.back().s)  return pts.back();

    // s가 속하는 구간 이분 탐색 (index 구하기)
    int lo = 0, hi = n - 1;
    while (hi - lo > 1) {
        int mid = (lo + hi) / 2;
        if (pts[mid].s <= s) lo = mid;
        else hi = mid;
    }

    double ds = pts[hi].s - pts[lo].s;
    double ratio = (ds > 1e-9) ? (s - pts[lo].s) / ds : 0.0;  // 선형 보간 비율 t값 구하기

    RefPoint rp;
    rp.s = s;
    rp.x = pts[lo].x + ratio * (pts[hi].x - pts[lo].x);
    rp.y = pts[lo].y + ratio * (pts[hi].y - pts[lo].y);
    rp.theta = pts[lo].theta + ratio * NormalizeAngleDiff(pts[hi].theta - pts[lo].theta);
    rp.kappa = pts[lo].kappa + ratio * (pts[hi].kappa - pts[lo].kappa);
    rp.d_kappa = pts[lo].d_kappa + ratio * (pts[hi].d_kappa - pts[lo].d_kappa);

    return rp;
}

// =========================================================
// FindClosestS — Newton-Raphson
//
// 논문 Appendix I은 [x - r(s)]^T * t_r(s) = 0  (원문 식, App.I 중간)
// 이 항상 성립한다고 명시한다 (트래킹 포인트에서 접선과 잔차 벡터가 직교).
// 즉 s* = argmin_s ||x - r(s)||  문제를, 아래의 스칼라 방정식의 근을
// 구하는 문제로 바꿀 수 있다.
//
//   f(s) := (r(s) - x)^T * t_r(s) = 0
//
// Newton-Raphson: s_{k+1} = s_k - f(s_k) / f'(s_k)
//
// f'(s)를 Frenet-Serret 공식 t_r'(s) = kappa_r(s) * n_r(s) 을 이용해
// 미분하면
//
//   f'(s) = t_r(s)^T t_r(s) + (r(s)-x)^T * kappa_r(s) * n_r(s)
//         = 1 + kappa_r(s) * [(r(s)-x)^T n_r(s)]
//
// 그런데 논문 식 (4)에서 d = (x - r(s))^T n_r(s) 로 정의하므로
// (r(s)-x)^T n_r(s) = -d 이고, 결국
//
//   f'(s) = 1 - kappa_r(s) * d
//
// 이는 근사식이 아니라 정확한 해석적 도함수다 (App.I 식(7) 분모에
// 등장하는 1 - kappa_r*d 항과 동일한 표현).
// =========================================================

double FindClosestS(const RefLine& ref, double x, double y) {

    // 초기값: 가장 가까운 웨이포인트의 s (brute-force)
    // Newton법은 국소 수렴이라 초기값이 실제 최소점 근방이어야 한다.
    double best_s = ref.points[0].s;
    double best_d2 = std::numeric_limits<double>::max();
    for (const auto& pt : ref.points) {
        double dx = x - pt.x;
        double dy = y - pt.y;
        double d2 = dx * dx + dy * dy;
        if (d2 < best_d2) { best_d2 = d2; best_s = pt.s; }
    }

    double s = best_s;
    double s_min = ref.points.front().s;
    double s_max = ref.points.back().s;

    for (int iter = 0; iter < 20; iter++) {
        RefPoint rp = Interpolate(ref, s);

        double tx = std::cos(rp.theta);   // t_r(s)
        double ty = std::sin(rp.theta);

        // f(s) = (r(s) - x)^T t_r(s)
        double f = (rp.x - x) * tx + (rp.y - y) * ty;

        // d = (x - r(s))^T n_r(s), n_r = (-sin theta, cos theta)
        double d = (x - rp.x) * (-std::sin(rp.theta)) + (y - rp.y) * std::cos(rp.theta);

        // f'(s) = 1 - kappa_r(s) * d
        double fp = 1.0 - rp.kappa * d;

        if (std::abs(fp) < 1e-9) break;  // 특이점(곡률 반경과 |d|가 거의 일치) 회피

        double ds = -f / fp;
        s += ds;

        // 탐색 범위를 벗어나면 클램핑 (외삽 방지)
        s = std::max(s_min, std::min(s, s_max));

        if (std::abs(ds) < 1e-6) break;  // 수렴
    }

    return s;
}
