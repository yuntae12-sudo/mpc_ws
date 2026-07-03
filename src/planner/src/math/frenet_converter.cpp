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
// CartesianToFrenet  (고속 모드 전용 — 헤더 주석 참고)
//
// s0는 FindClosestS(Newton-Raphson)로 구하고, kappa_x/a_x는 이미 입력으로
// 주어져 있으므로 식(4),(6),(9),(7),a_x식에 "대입"만 하면 되고 별도의
// 역산(비선형 풀이)이 필요 없다. 단, s_dot을 구하는 식(7) 역산과
// s_ddot을 구하는 a_x식 역산은 (1-kappa_r*d), cos(delta_theta), s_dot이
// 모두 0이 아니어야 하는 고속 모드 전제가 있다.
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
    // TODO(추후 개발 필요, FSM 저속 처리와 함께): cs.v ≈ 0 이면 s_dot ≈ 0이 되어
    // 아래 s_ddot 계산에서 0/0이 아니라 s_dot^2 항이 사라져 값 자체는 유한하지만,
    // "저속에서 재투영이 실제로 의미 있는가"는 FSM 설계 이후 다시 검토해야 함.
    s_dot = cs.v * cos_dtheta / one_minus_kd;

    // a_x 식을 s_ddot에 대해 정리
    const double rhs = cs.a - (s_dot * s_dot / cos_dtheta)
                        * (one_minus_kd * tan_dtheta * delta_theta_prime
                           - (dkappa_r * d + kappa_r * d_prime));
    s_ddot = rhs * cos_dtheta / one_minus_kd;
}
