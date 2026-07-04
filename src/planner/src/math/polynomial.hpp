#ifndef PLANNER_MATH_POLYNOMIAL_HPP
#define PLANNER_MATH_POLYNOMIAL_HPP

struct QuinticPolynomial {
    // 5차 다항식: Start[p0, v0, a0], End[p1, v1, a1] 모두 지정
    double a0, a1, a2, a3, a4, a5;

};

struct QuarticPolynomial {
    // 4차 다항식: End 위치 자유, End[v1, a1]만 지정 (Velocity Keeping)
    double a0, a1, a2, a3, a4;

};

// -------- 생성 (Free Function, 절차적 스타일 유지(?))

// xs = x(0), vs = x'(0), as = x''(0) (Start)
// xe = x(T), ve = x'(T), ae = x''(T) (End)
QuinticPolynomial MakeQuintic(double xs, double vs, double as,
                               double xe, double ve, double ae,
                               double T);

// Start 동일, End는 속도/가속도만
QuarticPolynomial MakeQuartic(double xs, double vs, double as,
                              double ve, double ae,
                              double T);

// 평가

double EvalPos (const QuinticPolynomial& p, double t);      // 위치
double EvalVel (const QuinticPolynomial& p, double t);      // 속도 x'
double EvalAcc (const QuinticPolynomial& p, double t);      // 가속도 x''
double EvalJerk (const QuinticPolynomial& p, double t);     // Jerk x'''

double EvalPos (const QuarticPolynomial& p, double t);
double EvalVel (const QuarticPolynomial& p, double t);
double EvalAcc (const QuarticPolynomial& p, double t);
double EvalJerk (const QuarticPolynomial& p, double t);

// =========================================================
// JerkCost: Jt(p(t)) := ∫[0,T] p'''(τ)^2 dτ  (Prop.1의 jerk 적분항)
//
// p'''(t)가 a3,a4(,a5)로만 이루어진 저차 다항식이라 적분을 해석적으로
// 미리 풀어놓은 닫힌 형태(closed form). 수치적분이 아니라 계수만으로
// 정확히 계산되며, cost.hpp가 아니라 다항식 계수가 살아있는 시점
// (path_generator가 MakeQuintic/MakeQuartic 호출한 직후)에 계산해서
// FrenetPath에 저장해두는 용도로 쓴다.
// =========================================================

double JerkCost(const QuinticPolynomial& p, double T);
double JerkCost(const QuarticPolynomial& p, double T);

#endif