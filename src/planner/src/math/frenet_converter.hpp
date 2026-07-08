#ifndef PLANNER_MATH_FRENET_CONVERTER_HPP
#define PLANNER_MATH_FRENET_CONVERTER_HPP

#include "frenet/ref_line.hpp"
#include "global/global.hpp"

// =========================================================
// 한 시점(instant)의 Cartesian 상태.
// 필드명은 CartesianPath와 맞춰 x, y, yaw, kappa, v, a로 통일.
// =========================================================

struct CartesianState {
    double x, y;
    double yaw;     // theta_x
    double kappa;   // kappa_x
    double v;       // v_x
    double a;       // a_x
};

// =========================================================
// [핵심 변환] Frenet -> Cartesian
// 논문 Appendix I 식 (1), (6), (9)를 이용한 변환.
// =========================================================

CartesianState FrenetToCartesian(const RefPoint& rp,
                                  double s, double s_dot, double s_ddot,
                                  double d, double d_prime, double d_pprime);

// =========================================================
// [고속 어댑터] (d, d_dot, d_ddot) -> (d, d', d'')
// 논문 App.I 마지막 문단: d' = d_dot / s_dot,  d'' = (d_ddot - d'*s_ddot) / s_dot^2
// s_dot != 0 인 고속 구간에서만 유효 (path_generator가 Sec.IV-A 방식,
// 즉 d(t) 다항식으로 후보를 생성했을 때 이 함수를 거쳐 핵심 변환으로 넘긴다).
//
// 호출 측에서 반드시 |s_dot| 이 충분히 큰지 확인 후 호출할 것
// (0 근방이면 저속 모드로 전환해서 d(s) 다항식을 직접 써야 함, Sec.IV-B).
// =========================================================

void TimeDerivToArcDeriv(double s_dot, double s_ddot,
                          double d_dot, double d_ddot,
                          double& d_prime, double& d_pprime);

// =========================================================
// [고속 어댑터의 역방향] (d, d', d'') -> (d_dot, d_ddot)
// 논문 App.I: d_dot = s_dot * d',  d_ddot = d''*s_dot^2 + d'*s_ddot
// =========================================================

void ArcDerivToTimeDeriv(double s_dot, double s_ddot,
                          double d_prime, double d_pprime,
                          double& d_dot, double& d_ddot);

// =========================================================
// [역변환] Cartesian -> Frenet
// 논문 Sec.VI 마지막 문단의 내용.
// 현재는 고속 모드 전용 (저속 처리는 추후 개발 필요).
// =========================================================

void CartesianToFrenet(const RefLine& ref, const CartesianState& cs,
                        double& s, double& s_dot, double& s_ddot,
                        double& d, double& d_prime, double& d_pprime);

// =========================================================
// [배치 변환] FrenetPath 전체 -> CartesianPath
// 선택된 FrenetPath(시간 샘플 배열)를 트래킹/MPC에 넘길 최종 CartesianPath (x,y,yaw,kappa,v,a 배열)로 변환한다.
//
// 전제: path는 FilterByCurvature/FilterByCollision을 통과한 valid==true
// 상태여야 한다. path.s_d[i]가 0에 가까운 샘플(STOP/EMERGENCY 정지 지점 등)은
// TimeDerivToArcDeriv 대신 ComputeGeometricPath 기반 값으로 대체해 안전하게
// 처리한다 (0/0=NaN 방지, .cpp 함수 상단 주석 참고).
// =========================================================

CartesianPath ConvertToCartesianPath(const FrenetPath& path, const RefLine& ref);

// =========================================================
// [저속 대응] 위치(x,y) 기반 기하학적 yaw/kappa 계산.
//
// x = r(s) + d*n_r(s) 는 s_dot과 무관하게 항상 well-defined이므로(식 (1)),
// FilterByCurvature/FilterByCollision이 쓰던 TimeDerivToArcDeriv 기반 계산
// (d_prime = d_dot/s_dot, s_dot->0에서 발산)을 거치지 않고 후보의 (x,y) 점
// 자체로부터 유한차분 yaw/kappa를 구한다. 저속(s_dot≈0) 후보(STOP/EMERGENCY
// 등)를 "몰라서 무효 처리"하지 않고 실제 기하로 정확히 판정할 수 있다.
// =========================================================

struct GeometricPath {
    std::vector<double> x, y, yaw, kappa;
};

GeometricPath ComputeGeometricPath(const std::vector<double>& s,
                                    const std::vector<double>& d,
                                    const RefLine& ref);

#endif
