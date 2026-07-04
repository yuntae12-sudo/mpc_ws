#include "frenet/ref_line.hpp"
#include <algorithm>
#include <cstdio>

namespace {

// theta는 atan2 결과라 ±π 경계에서 불연속(wrap-around)이 생김.
// 중앙/전후방 차분으로 kappa(=d_theta/ds)를 구할 때 이 불연속을 보정하지 않으면
// 곡선이 ±π를 지나는 순간 kappa가 튀는 버그가 생긴다.
double NormalizeAngleDiff(double dtheta) {
    while (dtheta > M_PI)  dtheta -= 2.0 * M_PI;
    while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
    return dtheta;
}

// =========================================================
// 원본 웨이포인트 전처리 (실측 CSV 검증 결과 반영)
//
// 실제 K-city path.txt를 오프라인으로 분석한 결과, 아래 두 유형의 노이즈가
// theta/kappa 중앙차분을 폭발시키는 것을 확인했다:
//   - 역행/중복 지터: 인접 두 점 사이 거리가 cm 단위로 매우 작은데 방향이
//     흔들려서 dtheta/ds의 분모가 0에 가까워짐 (예: 1.5cm 역행 후 재순행)
//   - 코너/직선부 저해상도: 실제 차량의 최소 회전반경(수 m)보다 훨씬 작은
//     회전반경(0.1m 등)이 나오는 구간 — 물리적으로 불가능하므로 기록 아티팩트
// 둘 다 "원본 샘플 간격이 불균일하고 노이즈가 섞여 있다"는 같은 근본 원인이라
// 최소 간격 필터 -> 호길이 기준 이동평균 스무딩 -> 균일 간격 재샘플링 순으로
// 해결한다. 이후 기존 중앙차분 theta/kappa/d_kappa 계산은 그대로 재사용.
// =========================================================

constexpr double kMinSpacing   = 0.05;  // [m] 이보다 가까운 점은 노이즈로 보고 버림
constexpr double kSmoothWindow = 0.5;   // [m] 호길이 기준 이동평균 윈도우 폭
constexpr double kResampleDs   = 0.1;   // [m] 최종 균일 재샘플링 간격

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

// 이전에 채택한 점과의 거리가 kMinSpacing 미만이면 버린다 (역행/중복 지터 제거).
void FilterMinSpacing(const std::vector<double>& wx, const std::vector<double>& wy,
                       std::vector<double>& out_x, std::vector<double>& out_y) {
    out_x.push_back(wx[0]);
    out_y.push_back(wy[0]);
    for (size_t i = 1; i < wx.size(); i++) {
        double dx = wx[i] - out_x.back();
        double dy = wy[i] - out_y.back();
        if (std::sqrt(dx * dx + dy * dy) >= kMinSpacing) {
            out_x.push_back(wx[i]);
            out_y.push_back(wy[i]);
        }
    }
}

// 각 점을 중심으로 호길이 ±window/2 안의 점들을 단순평균 (이동평균).
// 인덱스가 아니라 실제 거리 기준이라 점 밀도가 들쭉날쭉해도 물리적 스무딩 폭이 일정하다.
// window를 인자로 받는 이유: 기본 전처리(kSmoothWindow)는 트랙 전체에 약하게 걸고,
// FixInfeasibleCurvature가 "필렛으로 못 고친(=고립된 공백이 아니라 실제로 지속되는
// 커브에 노이즈가 얹힌) 구간"에서만 국소적으로 더 센 윈도우를 재시도할 때 재사용한다.
void SmoothByArcWindow(const std::vector<double>& wx, const std::vector<double>& wy,
                        double window,
                        std::vector<double>& out_x, std::vector<double>& out_y) {
    const int n = static_cast<int>(wx.size());
    std::vector<double> s = CumulativeArcLength(wx, wy);

    out_x.resize(n);
    out_y.resize(n);
    int lo = 0, hi = 0;
    for (int i = 0; i < n; i++) {
        while (s[i] - s[lo] > window / 2.0) lo++;
        while (hi < n - 1 && s[hi + 1] - s[i] <= window / 2.0) hi++;

        double sum_x = 0.0, sum_y = 0.0;
        for (int j = lo; j <= hi; j++) {
            sum_x += wx[j];
            sum_y += wy[j];
        }
        const int count = hi - lo + 1;
        out_x[i] = sum_x / count;
        out_y[i] = sum_y / count;
    }
}

// 스무딩된 폴리라인을 kResampleDs 간격으로 균일 재샘플링 (선형보간).
void ResampleUniform(const std::vector<double>& wx, const std::vector<double>& wy,
                      std::vector<double>& out_x, std::vector<double>& out_y) {
    const int n = static_cast<int>(wx.size());
    std::vector<double> s = CumulativeArcLength(wx, wy);
    const double total = s.back();

    int seg = 0;
    for (double target = 0.0; target < total - 1e-9; target += kResampleDs) {
        while (seg < n - 2 && s[seg + 1] < target) seg++;
        const double denom = s[seg + 1] - s[seg];
        const double ratio = (denom > 1e-9) ? (target - s[seg]) / denom : 0.0;
        out_x.push_back(wx[seg] + ratio * (wx[seg + 1] - wx[seg]));
        out_y.push_back(wy[seg] + ratio * (wy[seg + 1] - wy[seg]));
    }
    out_x.push_back(wx[n - 1]);
    out_y.push_back(wy[n - 1]);
}

// theta(중앙차분) + kappa(중앙차분, wrap-around 보정) 계산.
// BuildRefLine 최종 단계와 FixInfeasibleCurvature의 위반구간 탐지가 동일한
// 공식을 공유해야 하므로 하나로 뽑아둔다.
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

// =========================================================
// 3차 Hermite 곡선 (G1 필렛) — 위치 P0/P1, 접선방향 theta0/theta1을 만족하는
// 매끄러운 곡선. 접선 벡터 크기는 두 점 사이 직선거리(chord)로 스케일링
// (표준적인 선택 — Catmull-Rom과 동일한 관례).
// =========================================================
struct HermiteSegment {
    double p0x, p0y, t0x, t0y;
    double p1x, p1y, t1x, t1y;

    void Eval(double t, double& x, double& y, double& dx, double& dy,
              double& ddx, double& ddy) const {
        const double t2 = t * t, t3 = t2 * t;
        const double h00 = 2*t3 - 3*t2 + 1,  h10 = t3 - 2*t2 + t;
        const double h01 = -2*t3 + 3*t2,     h11 = t3 - t2;
        const double dh00 = 6*t2 - 6*t,      dh10 = 3*t2 - 4*t + 1;
        const double dh01 = -6*t2 + 6*t,     dh11 = 3*t2 - 2*t;
        const double ddh00 = 12*t - 6,       ddh10 = 6*t - 4;
        const double ddh01 = -12*t + 6,      ddh11 = 6*t - 2;

        x  = h00*p0x + h10*t0x + h01*p1x + h11*t1x;
        y  = h00*p0y + h10*t0y + h01*p1y + h11*t1y;
        dx = dh00*p0x + dh10*t0x + dh01*p1x + dh11*t1x;
        dy = dh00*p0y + dh10*t0y + dh01*p1y + dh11*t1y;
        ddx = ddh00*p0x + ddh10*t0x + ddh01*p1x + ddh11*t1x;
        ddy = ddh00*p0y + ddh10*t0y + ddh01*p1y + ddh11*t1y;
    }
};

double HermiteMaxCurvature(const HermiteSegment& seg, int samples) {
    double max_k = 0.0;
    for (int i = 0; i <= samples; i++) {
        double x, y, dx, dy, ddx, ddy;
        seg.Eval(static_cast<double>(i) / samples, x, y, dx, dy, ddx, ddy);
        const double denom = std::pow(dx*dx + dy*dy, 1.5);
        const double k = (denom > 1e-9) ? std::abs(dx*ddy - dy*ddx) / denom : 0.0;
        max_k = std::max(max_k, k);
    }
    return max_k;
}

// Hermite 곡선을 세밀히 샘플링해 실제 호길이를 구한 뒤, kResampleDs 간격에
// 맞춰 균일 재샘플링 (위치만 필요 — theta/kappa는 병합 후 ComputeThetaKappa로 재계산).
void SampleHermiteUniform(const HermiteSegment& seg, double ds,
                           std::vector<double>& out_x, std::vector<double>& out_y) {
    constexpr int kFineSamples = 200;
    std::vector<double> fx(kFineSamples + 1), fy(kFineSamples + 1), fs(kFineSamples + 1);
    fs[0] = 0.0;
    for (int i = 0; i <= kFineSamples; i++) {
        double x, y, dx, dy, ddx, ddy;
        seg.Eval(static_cast<double>(i) / kFineSamples, x, y, dx, dy, ddx, ddy);
        fx[i] = x; fy[i] = y;
        if (i > 0) fs[i] = fs[i - 1] + std::hypot(fx[i] - fx[i-1], fy[i] - fy[i-1]);
    }

    const double total = fs.back();
    const int n_out = std::max(1, static_cast<int>(std::round(total / ds)));
    int seg_idx = 0;
    for (int k = 0; k <= n_out; k++) {
        const double target = total * k / n_out;
        while (seg_idx < kFineSamples - 1 && fs[seg_idx + 1] < target) seg_idx++;
        const double denom = fs[seg_idx + 1] - fs[seg_idx];
        const double ratio = (denom > 1e-9) ? (target - fs[seg_idx]) / denom : 0.0;
        out_x.push_back(fx[seg_idx] + ratio * (fx[seg_idx+1] - fx[seg_idx]));
        out_y.push_back(fy[seg_idx] + ratio * (fy[seg_idx+1] - fy[seg_idx]));
    }
}

// 아이오닉5 최소회전반경 5.87m -> kappa_max=1/5.87=0.1704 에 10% 안전마진 적용.
constexpr double kMaxFeasibleCurvature = 0.153;
constexpr int kAnchorMarginStartPts = 5;    // 필렛 접선을 뽑을 여유 (포인트 개수, kResampleDs 기준)
constexpr int kAnchorMarginStepPts  = 10;
constexpr int kAnchorMarginMaxPts   = 150;  // 확장 한도 (~15m) — 넘으면 포기

// =========================================================
// TODO(기존 한계, 알고 있는 채로 남겨둠): 현재 K-city path.txt의 로터리
// 진출입부(약 s=2560~2591m, 트랙 전체의 약 1%)는 margin을 최대치까지
// 넓혀도 필렛으로 고쳐지지 않아 kMaxFeasibleCurvature 위반이 남는다
// (FixInfeasibleCurvature가 자동으로 fprintf(stderr,...) 경고를 남김).
//
// 원인: 이 구간은 고립된 데이터 공백이 아니라 실제로 계속 굽어있는 실제
// 도로(로터리)라서, 양옆 접선을 이어 붙이는 필렛 방식 자체가 안 맞는다
// (필렛을 놓으면 실제 도로를 가로질러 잘라먹게 됨). 국소 재스무딩으로
// 우회 시도했으나 splice 경계에서 위치/접선이 안 맞아 오히려 새 곡률
// 스파이크를 만들어내는 발산 루프에 빠져 롤백했다 (2026-07 확인).
//
// 로터리는 실제로도 저속 주행 구간이라, 논문 Sec.IV-B(저속 모드, d(s)
// 다항식 + 비홀로노믹 곡률 제약)가 구현되기 전까지는 이 planner의
// 스코프(고속 quintic 후보 생성) 밖에 있는 한계로 보고 넘어간다.
// 제대로 고치려면 곡률 상한을 명시적 제약으로 넣는 최적화 기반
// 스무딩(QP 등)이 필요.
// =========================================================

// =========================================================
// 급격한 곡률 구간(=원본 GPS 로깅 공백으로 실제보다 급하게 꺾여 기록된 지점)을
// 자동 탐지해서 접선연속(G1) Hermite 필렛으로 대체한다.
//
// 실측(K-city path.txt) 결과, 이런 지점은 이동평균 스무딩 윈도우를 아무리
// 키워도(4m까지 확인) 회전반경이 1m 근처에서 정체된다 — 데이터가 왜곡된 게
// 아니라 애초에 누락된 것이라 국소 평균으로는 복원되지 않기 때문. 정상 구간
// (위반구간 양옆)의 접선 방향은 그대로 살려서 필렛 경계를 정하고, 그 사이를
// kMaxFeasibleCurvature 이내로 들어올 때까지 필렛 구간을 점점 넓히며 재시도한다.
// =========================================================
void FixInfeasibleCurvature(std::vector<double>& wx, std::vector<double>& wy) {
    std::vector<double> theta, kappa;
    ComputeThetaKappa(wx, wy, theta, kappa);

    int n = static_cast<int>(wx.size());
    int i = 0;
    while (i < n) {
        if (std::abs(kappa[i]) <= kMaxFeasibleCurvature) { i++; continue; }

        int lo = i, hi = i;
        while (hi + 1 < n && std::abs(kappa[hi + 1]) > kMaxFeasibleCurvature) hi++;

        bool fixed = false;
        for (int margin = kAnchorMarginStartPts; margin <= kAnchorMarginMaxPts;
             margin += kAnchorMarginStepPts) {
            const int a = lo - margin;
            const int b = hi + margin;
            if (a < 0 || b >= n) break;  // 트랙 경계 근처 -> 확장 불가, 포기

            const double chord = std::hypot(wx[b] - wx[a], wy[b] - wy[a]);
            HermiteSegment seg;
            seg.p0x = wx[a]; seg.p0y = wy[a];
            seg.p1x = wx[b]; seg.p1y = wy[b];
            seg.t0x = chord * std::cos(theta[a]); seg.t0y = chord * std::sin(theta[a]);
            seg.t1x = chord * std::cos(theta[b]); seg.t1y = chord * std::sin(theta[b]);

            if (HermiteMaxCurvature(seg, 200) > kMaxFeasibleCurvature) continue;  // 더 넓혀서 재시도

            std::vector<double> fill_x, fill_y;
            SampleHermiteUniform(seg, kResampleDs, fill_x, fill_y);

            std::vector<double> new_x(wx.begin(), wx.begin() + a);
            std::vector<double> new_y(wy.begin(), wy.begin() + a);
            new_x.insert(new_x.end(), fill_x.begin(), fill_x.end());
            new_y.insert(new_y.end(), fill_y.begin(), fill_y.end());
            new_x.insert(new_x.end(), wx.begin() + b + 1, wx.end());
            new_y.insert(new_y.end(), wy.begin() + b + 1, wy.end());

            wx = std::move(new_x);
            wy = std::move(new_y);
            fixed = true;
            break;
        }

        // margin을 kAnchorMarginMaxPts까지 넓혀도 kMaxFeasibleCurvature를 못 맞췄거나
        // 트랙 경계에 부딪혀 확장 자체가 불가능했던 경우. 이 위반 구간은 그대로 남아서
        // FilterByCurvature가 이 근방을 지나는 모든 후보를 계속 무효 처리하게 되는데,
        // 원인을 다시 처음부터 찾지 않도록 실패 사실 자체를 로그로 남긴다.
        //
        // (시도했다가 되돌린 방법: 이 구간만 국소적으로 더 큰 윈도우로 재스무딩해서
        // 덮어쓰는 방식 — 결과가 오히려 악화됨. resmoothed 값을 [lo,hi]에만 잘라
        // 넣으면 그 경계(lo-1↔lo, hi↔hi+1)에서 위치가 매끄럽게 안 이어져 새로운
        // 곡률 스파이크가 생기고, 그걸 다음 반복이 또 위반으로 잡아 계속 악화되는
        // 발산 루프에 빠졌다. 접선까지 매끄럽게 잇는 블렌딩 없이는 이 방식 자체가
        // 성립하지 않으므로, 제대로 된 방법이 정해지기 전까지는 시도하지 않는다.)
        if (!fixed) {
            std::fprintf(stderr,
                "[BuildRefLine] WARNING: infeasible curvature at idx=[%d,%d] xy=(%.3f,%.3f) "
                "(kappa up to %.4f) could not be auto-fixed within margin cap (%dpt) — left as-is\n",
                lo, hi, wx[lo], wy[lo], kappa[i], kAnchorMarginMaxPts);
        }

        // 배열 크기가 바뀌었을 수 있으므로 theta/kappa를 다시 계산하고,
        // 고쳤으면 처음부터, 못 고쳤으면 이 구간 다음부터 계속 탐색.
        ComputeThetaKappa(wx, wy, theta, kappa);
        n = static_cast<int>(wx.size());
        i = fixed ? 0 : (hi + 1);
    }
}

}  // namespace

// =========================================================
// BuildRefLine
// =========================================================

RefLine BuildRefLine(const std::vector<double>& wx_raw, const std::vector<double>& wy_raw) {

    if (wx_raw.size() < 2 || wx_raw.size() != wy_raw.size())
        throw std::invalid_argument("Waypoints must have at least 2 points and matching sizes");

    std::vector<double> wx_filtered, wy_filtered, wx_smoothed, wy_smoothed, wx, wy;
    FilterMinSpacing(wx_raw, wy_raw, wx_filtered, wy_filtered);
    SmoothByArcWindow(wx_filtered, wy_filtered, kSmoothWindow, wx_smoothed, wy_smoothed);
    ResampleUniform(wx_smoothed, wy_smoothed, wx, wy);
    FixInfeasibleCurvature(wx, wy);

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

    // 2~3. theta/kappa: 중앙 차분 (wrap-around 보정 포함, FixInfeasibleCurvature와 동일 공식 재사용)
    std::vector<double> theta, kappa;
    ComputeThetaKappa(wx, wy, theta, kappa);
    for (int i = 0; i < n; i++) {
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
