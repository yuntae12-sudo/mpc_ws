#include "frenet_planner/frenet/ref_line.hpp"
#include <algorithm>

namespace {

double NormalizeAngleDiff(double dtheta) {
    while (dtheta > M_PI)  dtheta -= 2.0 * M_PI;
    while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
    return dtheta;
}

std::vector<double> CumulativeArcLength(const std::vector<double>& x, const std::vector<double>& y) {
    std::vector<double> s(x.size());
    s[0] = 0.0;
    for (size_t i = 1; i < x.size(); i++) {
        double dx = x[i] - x[i - 1];
        double dy = y[i] - y[i - 1];
        s[i] = s[i - 1] + std::sqrt(dx * dx + dy * dy);
    }
    return s;
}

// (아래 3단계 스무딩 파이프라인에서 사용, 정의는 이 파일 뒤쪽에 있음)
void ComputeThetaKappa(const std::vector<double>& x, const std::vector<double>& y,
                        std::vector<double>& theta, std::vector<double>& kappa);

// MGeo 링크를 이어붙인 원본 waypoint에는 국소적으로 물리적 회전반경(1~2m대)을
// 넘는 급격한 꺾임이 다수 섞여있어(링크 이어붙임 이음매, 원본 좌표 노이즈 등),
// FilterByCurvature가 그 지점을 지나는 모든 후보를 동시에 무효화시킨다.
// 아래 3단계(스무딩 -> 균일 재샘플링 -> 잔여 초과분 국소 이완)로 완화한다.
// 스무딩만 단독으로 하면 인접점끼리 arc-length상 다닥다닥 붙어버려 오히려
// kappa=dtheta/ds의 분모(ds)가 작아지며 곡률이 더 커지는 역효과가 실측으로
// 확인됐다 (재샘플링으로 ds를 다시 균일하게 맞춰줘야 함).
constexpr double kSmoothingWindowM = 10.0;  // [m] 이동평균 전체 윈도우 폭
constexpr double kResampleDs       = 0.5;   // [m] 재샘플링 간격 (원본 waypoint 간격과 동일)
constexpr int    kRelaxMaxIter     = 300;
constexpr double kRelaxStep        = 0.3;   // 초과 지점을 양옆 중점 쪽으로 당기는 비율

void SmoothWaypoints(const std::vector<double>& x, const std::vector<double>& y,
                      double window_m, std::vector<double>& out_x, std::vector<double>& out_y) {
    const int n = static_cast<int>(x.size());
    std::vector<double> s = CumulativeArcLength(x, y);
    out_x.resize(n);
    out_y.resize(n);

    int lo = 0, hi = 0;
    double sum_x = 0.0, sum_y = 0.0;
    for (int i = 0; i < n; i++) {
        const double s_lo = s[i] - window_m / 2.0;
        const double s_hi = s[i] + window_m / 2.0;

        while (lo < n && s[lo] < s_lo) { sum_x -= x[lo]; sum_y -= y[lo]; lo++; }
        while (hi < n && s[hi] <= s_hi) { sum_x += x[hi]; sum_y += y[hi]; hi++; }

        const int cnt = hi - lo;
        out_x[i] = (cnt > 0) ? sum_x / cnt : x[i];
        out_y[i] = (cnt > 0) ? sum_y / cnt : y[i];
    }
}

// 스무딩으로 흐트러진 점 간격을 균일 arc-length 간격으로 재조정.
void ResampleUniform(const std::vector<double>& x, const std::vector<double>& y, double ds_target,
                      std::vector<double>& out_x, std::vector<double>& out_y) {
    std::vector<double> s = CumulativeArcLength(x, y);
    const double total = s.back();
    const int n_out = static_cast<int>(total / ds_target) + 1;
    out_x.resize(n_out);
    out_y.resize(n_out);

    int j = 0;
    for (int k = 0; k < n_out; k++) {
        const double target_s = k * ds_target;
        while (j < static_cast<int>(s.size()) - 2 && s[j + 1] < target_s) j++;
        const double ds = s[j + 1] - s[j];
        const double ratio = (ds > 1e-9) ? (target_s - s[j]) / ds : 0.0;
        out_x[k] = x[j] + ratio * (x[j + 1] - x[j]);
        out_y[k] = y[j] + ratio * (y[j + 1] - y[j]);
    }
}

// 스무딩+재샘플링 이후에도 남아있는 국소 곡률 초과 지점을, 양옆 점의 중점
// 쪽으로 조금씩 당겨서(라플라시안 완화) 반복적으로 줄인다.
void FixInfeasibleCurvature(std::vector<double>& x, std::vector<double>& y, double max_curvature) {
    const int n = static_cast<int>(x.size());
    if (n < 3) return;

    std::vector<double> theta, kappa;
    for (int iter = 0; iter < kRelaxMaxIter; iter++) {
        ComputeThetaKappa(x, y, theta, kappa);

        std::vector<int> bad;
        for (int i = 1; i < n - 1; i++) {
            if (std::abs(kappa[i]) > max_curvature) bad.push_back(i);
        }
        if (bad.empty()) break;

        for (int i : bad) {
            const double mx = 0.5 * (x[i - 1] + x[i + 1]);
            const double my = 0.5 * (y[i - 1] + y[i + 1]);
            x[i] = x[i] * (1.0 - kRelaxStep) + mx * kRelaxStep;
            y[i] = y[i] * (1.0 - kRelaxStep) + my * kRelaxStep;
        }
    }
}

// theta(중앙차분) + kappa(중앙차분, wrap-around 보정) 계산.
void ComputeThetaKappa(const std::vector<double>& x, const std::vector<double>& y,
                        std::vector<double>& theta, std::vector<double>& kappa) {
    const int n = static_cast<int>(x.size());
    std::vector<double> s = CumulativeArcLength(x, y);

    theta.resize(n);
    for (int i = 0; i < n; i++) {
        double dx, dy;
        if (i == 0)          { dx = x[1] - x[0];         dy = y[1] - y[0]; }
        else if (i == n - 1) { dx = x[n-1] - x[n-2];      dy = y[n-1] - y[n-2]; }
        else                 { dx = x[i+1] - x[i-1];      dy = y[i+1] - y[i-1]; }
        theta[i] = std::atan2(dy, dx);
    }

    kappa.resize(n);
    for (int i = 0; i < n; i++) {
        const int a = (i == 0) ? 0 : i - 1;
        const int b = (i == n - 1) ? n - 1 : i + 1;
        const double d_theta = NormalizeAngleDiff(theta[b] - theta[a]);
        const double ds = s[b] - s[a];
        kappa[i] = (ds > 1e-9) ? d_theta / ds : 0.0;
    }
}

}  // namespace

// =========================================================
// BuildRefLine
// =========================================================

RefLine BuildRefLine(const std::vector<double>& wx, const std::vector<double>& wy, double max_curvature) {

    if (wx.size() < 2 || wx.size() != wy.size())
        throw std::invalid_argument("Waypoints must have at least 2 points and matching sizes");

    // 0. 국소 급커브 완화: 스무딩 -> 균일 재샘플링 -> 잔여 초과분 국소 이완.
    std::vector<double> smoothed_x, smoothed_y;
    SmoothWaypoints(wx, wy, kSmoothingWindowM, smoothed_x, smoothed_y);

    std::vector<double> sx, sy;
    ResampleUniform(smoothed_x, smoothed_y, kResampleDs, sx, sy);

    FixInfeasibleCurvature(sx, sy, max_curvature);

    RefLine ref;
    int n = static_cast<int>(sx.size());
    ref.points.resize(n);

    // 1. arc length s 누적 계산
    std::vector<double> s = CumulativeArcLength(sx, sy);

    // 2~3. theta/kappa: 중앙 차분 (wrap-around 보정 포함)
    std::vector<double> theta, kappa;
    ComputeThetaKappa(sx, sy, theta, kappa);

    for (int i = 0; i < n; i++) {
        ref.points[i].x = sx[i];
        ref.points[i].y = sy[i];
        ref.points[i].s = s[i];
        ref.points[i].theta = theta[i];
        ref.points[i].kappa = kappa[i];
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

    // 폐루프(닫힌 트랙) wrap-around: 이전엔 s를 [front().s, back().s]로
    // 클램핑했는데, 이러면 이음새(피니시라인) 근처에서 s가 끝점에 얼어붙어
    // 실제 위치와 무관한 d/heading이 나오는 문제가 있었다(실측: 마지막 구간
    // 경로추종 불안정). front()==back() 위치가 물리적으로 같은 점(폐루프
    // 폐합점)이므로, 범위를 벗어나면 클램핑 대신 [0, s_max)로 감아준다.
    const double s_max = pts.back().s;
    double s_wrapped = std::fmod(s, s_max);
    if (s_wrapped < 0.0) s_wrapped += s_max;
    s = s_wrapped;

    // 부동소수 오차로 경계값을 살짝 벗어나는 경우만 클램핑.
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

double FindClosestS(const RefLine& ref, double x, double y,
                     const double* s_hint, double window) {

    // 초기값: 가장 가까운 웨이포인트의 s (brute-force).
    // Newton법은 국소 수렴이라 초기값이 실제 최소점 근방이어야 한다.
    //
    // s_hint가 있으면 s_hint ± window 범위의 점들만 후보로 삼는다 (헤더 주석
    // 참고 - 트랙이 자기 자신과 가까운 구간에서 전체 탐색이 엉뚱하게 먼 s로
    // 튀는 문제 방지). 범위 안에 점이 하나도 없으면(비정상 상황) 안전하게
    // 전체 탐색으로 폴백한다.
    const double s_max_for_wrap = ref.points.back().s;

    double best_s = ref.points[0].s;
    double best_d2 = std::numeric_limits<double>::max();
    bool found = false;
    if (s_hint) {
        // 폐루프 wrap-around: 단순 [hint-window, hint+window] 범위 체크는
        // hint가 이음새(s≈0 또는 s≈s_max) 근처일 때 반대편(물리적으로는
        // 바로 붙어있는) 구간의 점들을 전부 놓친다. s축 위의 최短 원형
        // 거리(circular distance)로 비교해 이음새를 넘어서도 창이 정상
        // 동작하게 한다.
        for (const auto& pt : ref.points) {
            double diff = std::fabs(pt.s - *s_hint);
            double circ_dist = std::min(diff, s_max_for_wrap - diff);
            if (circ_dist > window) continue;
            double dx = x - pt.x;
            double dy = y - pt.y;
            double d2 = dx * dx + dy * dy;
            if (d2 < best_d2) { best_d2 = d2; best_s = pt.s; found = true; }
        }
    }
    if (!found) {
        for (const auto& pt : ref.points) {
            double dx = x - pt.x;
            double dy = y - pt.y;
            double d2 = dx * dx + dy * dy;
            if (d2 < best_d2) { best_d2 = d2; best_s = pt.s; }
        }
    }

    double s = best_s;
    const double s_max = s_max_for_wrap;

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

        // 폐루프 wrap-around: 이음새 근처에서 Newton 스텝이 s=0 또는
        // s=s_max를 자연스럽게 넘나들 수 있어야 한다(클램핑하면 반대편으로
        // 못 건너가고 끝점에 갇힌다). Interpolate도 동일하게 wrap하므로
        // f(s)/f'(s) 계산 자체는 항상 유효하다.
        s = std::fmod(s, s_max);
        if (s < 0.0) s += s_max;

        if (std::abs(ds) < 1e-6) break;  // 수렴
    }

    return s;
}
