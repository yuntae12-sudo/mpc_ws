#include "math/frenet_converter.hpp"

namespace {

double NormalizeAngle(double theta) {
    while (theta > M_PI)  theta -= 2.0 * M_PI;
    while (theta < -M_PI) theta += 2.0 * M_PI;
    return theta;
}

}  // namespace

// =========================================================
// FrenetToCartesian
// 식 (1): x = r(s) + d * n_r(s)
// 식 (6): d' = (1 - kappa_r*d) * tan(delta_theta)  ->  delta_theta = atan2(d', 1-kappa_r*d)
// 식 (9): d''을 kappa_x에 대해 풀어서 얻은 (Apollo류로 잘 알려진) 닫힌 형태
// 식 (7): v_x = s_dot * (1-kappa_r*d) / cos(delta_theta)
// App.I 말미 a_x 식
// =========================================================

CartesianState FrenetToCartesian(const RefPoint& rp,
                                  double s, double s_dot, double s_ddot,
                                  double d, double d_prime, double d_pprime) {
    (void)s;  // rp가 이미 s에서 보간된 값이므로 s 자체는 위치 계산에 쓰지 않음

    const double theta_r  = rp.theta;
    const double kappa_r  = rp.kappa;
    const double dkappa_r = rp.d_kappa;

    const double tx = std::cos(theta_r);
    const double ty = std::sin(theta_r);
    const double nx = -std::sin(theta_r);
    const double ny =  std::cos(theta_r);

    CartesianState cs;

    // 식 (1)
    cs.x = rp.x + d * nx;
    cs.y = rp.y + d * ny;

    const double one_minus_kd = 1.0 - kappa_r * d;

    // 식 (6): delta_theta
    const double delta_theta = std::atan2(d_prime, one_minus_kd);
    const double cos_dtheta  = std::cos(delta_theta);
    const double tan_dtheta  = std::tan(delta_theta);

    cs.yaw = NormalizeAngle(theta_r + delta_theta);

    // 식 (9)를 kappa_x에 대해 정리한 닫힌 형태:
    // kappa_x = cos(dtheta)/(1-kr d) * kr
    //         + cos^3(dtheta)/(1-kr d)^2 * [d'' + (kr'*d + kr*d')*tan(dtheta)]
    const double bracket = d_pprime + (dkappa_r * d + kappa_r * d_prime) * tan_dtheta;
    cs.kappa = kappa_r * cos_dtheta / one_minus_kd
             + bracket * cos_dtheta * cos_dtheta * cos_dtheta / (one_minus_kd * one_minus_kd);

    // 식 (7)
    cs.v = s_dot * one_minus_kd / cos_dtheta;

    // delta_theta' = kappa_x*(1-kr d)/cos(dtheta) - kr   (식(9) 유도 과정에 등장하는 항, 재사용)
    const double delta_theta_prime = cs.kappa * one_minus_kd / cos_dtheta - kappa_r;

    // App.I 말미 a_x 식
    cs.a = s_ddot * one_minus_kd / cos_dtheta
         + (s_dot * s_dot / cos_dtheta)
           * (one_minus_kd * tan_dtheta * delta_theta_prime - (dkappa_r * d + kappa_r * d_prime));

    return cs;
}

// =========================================================
// TimeDerivToArcDeriv / ArcDerivToTimeDeriv
// App.I 말미: d_dot = s_dot * d',  d_ddot = d''*s_dot^2 + d'*s_ddot
// =========================================================

void TimeDerivToArcDeriv(double s_dot, double s_ddot,
                          double d_dot, double d_ddot,
                          double& d_prime, double& d_pprime) {
    // 호출 전 |s_dot|이 충분히 큰지(고속 모드) 확인하는 것은 호출 측 책임
    d_prime  = d_dot / s_dot;
    d_pprime = (d_ddot - d_prime * s_ddot) / (s_dot * s_dot);
}

void ArcDerivToTimeDeriv(double s_dot, double s_ddot,
                          double d_prime, double d_pprime,
                          double& d_dot, double& d_ddot) {
    d_dot  = s_dot * d_prime;
    d_ddot = d_pprime * s_dot * s_dot + d_prime * s_ddot;
}

// =========================================================
// CartesianToFrenet
//
// s0는 FindClosestS(Newton-Raphson)로 구하고, kappa_x/a_x는 이미 입력으로
// 주어져 있으므로 식(4),(6),(9),(7),a_x식에 "대입"만 하면 되고 별도의
// 역산(비선형 풀이)이 필요 없다.
//
// 저속(v≈0)에서도 안전: 나눗셈 분모는 (1-kappa_r*d)와 cos(delta_theta)뿐이고
// 둘 다 속도와 무관한 순수 기하량(d가 곡률 중심과 겹치는지, 차량이 경로와
// 수직으로 틀어졌는지)이다. s_dot(=cs.v*cos_dtheta/one_minus_kd)은 분모가
// 아니라 분자로만 등장해서, v=0이면 s_dot=0이 0/0이 아니라 깔끔하게 나오고
// 이어지는 s_ddot 계산의 s_dot^2 항도 그대로 소거된다.
// =========================================================

void CartesianToFrenet(const RefLine& ref, const CartesianState& cs,
                        double& s, double& s_dot, double& s_ddot,
                        double& d, double& d_prime, double& d_pprime) {

    s = FindClosestS(ref, cs.x, cs.y);
    RefPoint rp = Interpolate(ref, s);

    const double theta_r  = rp.theta;
    const double kappa_r  = rp.kappa;
    const double dkappa_r = rp.d_kappa;

    // 식 (4): d = (x - r(s))^T n_r(s)
    d = (cs.x - rp.x) * (-std::sin(theta_r)) + (cs.y - rp.y) * std::cos(theta_r);

    const double one_minus_kd = 1.0 - kappa_r * d;
    const double delta_theta  = NormalizeAngle(cs.yaw - theta_r);
    const double cos_dtheta   = std::cos(delta_theta);
    const double tan_dtheta   = std::tan(delta_theta);

    // 식 (6)
    d_prime = one_minus_kd * tan_dtheta;

    // delta_theta' = kappa_x*(1-kr d)/cos(dtheta) - kr  (kappa_x는 입력 cs.kappa로 이미 알고 있음)
    const double delta_theta_prime = cs.kappa * one_minus_kd / cos_dtheta - kappa_r;

    // 식 (9)를 d''에 대해 정리 (kappa_x가 이미 주어져 있으므로 순방향 대입)
    d_pprime = -(dkappa_r * d + kappa_r * d_prime) * tan_dtheta
             + (one_minus_kd / (cos_dtheta * cos_dtheta)) * delta_theta_prime;

    // 식 (7)을 s_dot에 대해 정리: v_x = s_dot*(1-kr d)/cos(dtheta)
    s_dot = cs.v * cos_dtheta / one_minus_kd;

    // a_x 식을 s_ddot에 대해 정리
    const double rhs = cs.a - (s_dot * s_dot / cos_dtheta)
                        * (one_minus_kd * tan_dtheta * delta_theta_prime
                           - (dkappa_r * d + kappa_r * d_prime));
    s_ddot = rhs * cos_dtheta / one_minus_kd;
}

// ConvertToCartesianPath (설계 노트는 헤더 참고)
//
// STOP/EMERGENCY 후보는 마지막 샘플에서 s_dot=0, s_ddot=0, d_dot=0, d_ddot=0이
// 되도록 설계된다 (완전 정지). 이때 TimeDerivToArcDeriv의 d_prime=d_dot/s_dot는
// 0/0 = NaN이 되고, 그 NaN이 FrenetToCartesian의 yaw/kappa/v/a 전부를 오염시킨다
// (진짜 값은 전부 0이지만 "0 * NaN = NaN"으로 뒤덮임). 저속 구간만 골라
// 안전한 값(위치 기반 yaw/kappa, s_dot/s_ddot을 그대로 v/a로)으로 대체한다.
CartesianPath ConvertToCartesianPath(const FrenetPath& path, const RefLine& ref) {
    CartesianPath result;

    const size_t n = path.t.size();
    result.x.reserve(n);
    result.y.reserve(n);
    result.yaw.reserve(n);
    result.kappa.reserve(n);
    result.v.reserve(n);
    result.a.reserve(n);

    constexpr double kMinSpeedForAnalytic = 0.1;  // [m/s]

    GeometricPath geo = ComputeGeometricPath(path.s, path.d, ref);

    for (size_t i = 0; i < n; i++) {
        if (std::abs(path.s_d[i]) < kMinSpeedForAnalytic) {
            result.x.push_back(geo.x[i]);
            result.y.push_back(geo.y[i]);
            result.yaw.push_back(geo.yaw[i]);
            result.kappa.push_back(geo.kappa[i]);
            result.v.push_back(path.s_d[i]);
            result.a.push_back(path.s_dd[i]);
            continue;
        }

        double d_prime, d_pprime;
        TimeDerivToArcDeriv(path.s_d[i], path.s_dd[i], path.d_d[i], path.d_dd[i],
                             d_prime, d_pprime);

        RefPoint rp = Interpolate(ref, path.s[i]);
        CartesianState cs = FrenetToCartesian(rp, path.s[i], path.s_d[i], path.s_dd[i],
                                               path.d[i], d_prime, d_pprime);

        result.x.push_back(cs.x);
        result.y.push_back(cs.y);
        result.yaw.push_back(cs.yaw);
        result.kappa.push_back(cs.kappa);
        result.v.push_back(cs.v);
        result.a.push_back(cs.a);
    }

    return result;
}

// ComputeGeometricPath (설계 노트는 헤더 참고)
GeometricPath ComputeGeometricPath(const std::vector<double>& s,
                                    const std::vector<double>& d,
                                    const RefLine& ref) {
    const size_t n = s.size();
    GeometricPath gp;
    gp.x.resize(n);
    gp.y.resize(n);
    gp.yaw.resize(n, 0.0);
    gp.kappa.resize(n, 0.0);

    // 식 (1): x = r(s) + d*n_r(s). s_dot/d_prime과 무관하므로 항상 안전.
    // ref_theta는 저속 구간의 yaw fallback으로 재사용 (아래 참고).
    std::vector<double> ref_theta(n);
    for (size_t i = 0; i < n; i++) {
        RefPoint rp = Interpolate(ref, s[i]);
        gp.x[i] = rp.x + d[i] * (-std::sin(rp.theta));
        gp.y[i] = rp.y + d[i] * std::cos(rp.theta);
        ref_theta[i] = rp.theta;
    }

    // 이 거리보다 짧은 구간은 저속으로 점들이 거의 겹쳐서 방향을 못 믿는다 -
    // "무효 처리"가 아니라 "판단 보류"로 참조선 접선방향(ref_theta)을 대신 쓴다.
    // 예전엔 "직전 값을 이어받되 i==0이면 0.0"으로 처리했는데, 이게 정지 상태에서
    // 출발하는(v=0에서 가속하는) 후보의 초반 몇 샘플처럼 "직전 값이 아예 없는" 경우
    // 하드코딩된 0.0을 실제 방향(예: 90도)과 무관하게 그대로 유지하다가, 속도가
    // 붙어 진짜 위치 기반 yaw가 계산되는 순간 그 사이에서 가짜 급곡률(kappa)이
    // 튀어나와 FilterByCurvature가 출발 후보를 전부 무효화하는 버그가 있었다
    // (실차 검증으로 확인 - 정지 상태에서 차가 아예 출발을 못 함). ref_theta는
    // s[i]마다 항상 well-defined해서 "직전 값 의존" 문제 자체가 없다.
    //
    // 원래 0.02였는데, 이번엔 반대쪽(STOP 직전 저속 tail, v=0.1~0.7m/s)에서
    // 세그먼트 길이가 0.02보다는 크지만(0.03~0.13m) 위치 노이즈가 그대로
    // kappa로 증폭되는 걸 실측으로 확인함(kappa=3.1까지, 회전반경 0.3m라는
    // 뜻이라 명백한 수치 오류). 실측된 실패 구간(최대 0.13m)을 여유 있게
    // 덮도록 0.2로 올림 - 실제 커브 구간(seg_len 0.7m+)에는 영향 없음.
    constexpr double kMinSegmentLength = 0.2;  // [m]

    for (size_t i = 0; i < n; i++) {
        double dx, dy;
        if (i == 0)          { dx = gp.x[1] - gp.x[0];       dy = gp.y[1] - gp.y[0]; }
        else if (i == n - 1) { dx = gp.x[n-1] - gp.x[n-2];   dy = gp.y[n-1] - gp.y[n-2]; }
        else                 { dx = gp.x[i+1] - gp.x[i-1];   dy = gp.y[i+1] - gp.y[i-1]; }

        if (std::hypot(dx, dy) > kMinSegmentLength) {
            gp.yaw[i] = std::atan2(dy, dx);
        } else {
            gp.yaw[i] = ref_theta[i];
        }
    }

    for (size_t i = 0; i < n; i++) {
        const size_t a = (i == 0) ? 0 : i - 1;
        const size_t b = (i == n - 1) ? n - 1 : i + 1;
        const double seg_len = std::hypot(gp.x[b] - gp.x[a], gp.y[b] - gp.y[a]);
        if (seg_len > kMinSegmentLength) {
            const double d_theta = NormalizeAngle(gp.yaw[b] - gp.yaw[a]);
            gp.kappa[i] = d_theta / seg_len;
        }
        // 짧은 구간은 kappa=0(초기값)으로 남겨 "판단 보류" (안전 쪽으로: 통과시킴).
    }

    return gp;
}
