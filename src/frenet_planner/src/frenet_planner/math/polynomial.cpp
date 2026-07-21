#include "polynomial.hpp"

QuinticPolynomial MakeQuintic(double xs, double vs, double as,
                              double xe, double ve, double ae,
                              double T)
{
    QuinticPolynomial p;
    p.a0 = xs;
    p.a1 = vs;
    p.a2 = as / 2.0;

    double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;

    double c0 = xe - (p.a0 + p.a1 * T + p.a2 * T2);
    double c1 = ve - (p.a1 + 2.0 * p.a2 * T);
    double c2 = ae - (2.0 * p.a2);

    p.a3 = (10.0 * c0 - 4.0 * c1 * T + 0.5 * c2 * T2) / T3;
    p.a4 = (-15.0 * c0 + 7.0 * c1 * T - c2 * T2) / T4;
    p.a5 = (6.0 * c0 - 3.0 * c1 * T + 0.5 * c2 * T2) /T5;

    return p;
}

QuarticPolynomial MakeQuartic(double xs, double vs, double as,
                              double ve, double ae,
                              double T)
{
    QuarticPolynomial p;
    p.a0 = xs;
    p.a1 = vs;
    p.a2 = as / 2.0;

    double T2 = T * T, T3 = T2 * T, T4 = T3 * T;

    // End 위치 자유 -> a3, a4만 2x2로 풀림
    p.a3 = (ve - p.a1 - (4.0/3.0) * p.a2 * T - (1.0/3.0) * ae * T) / T2;
    p.a4 = (ae - 2.0 * p.a2 - 6.0 * p.a3 * T) / (12.0 * T2);

    return p;
}

// ================= Quintic 평가 ========================

double EvalPos(const QuinticPolynomial& p, double t) {

    return p.a0 + p.a1 * t + p.a2 * t * t + p.a3 * t * t * t
         + p.a4 * t * t * t * t + p.a5 * t * t * t * t * t;

}

double EvalVel(const QuinticPolynomial& p, double t) {

    return p.a1 + 2.0 * p.a2 * t + 3.0 * p.a3 * t * t 
        + 4.0 * p.a4 * t * t * t + 5.0 * p.a5 * t * t * t * t;

}

double EvalAcc(const QuinticPolynomial& p, double t) {

    return 2.0 * p.a2 + 6.0 * p.a3 * t
        + 12.0 * p.a4 * t * t + 20.0 * p.a5 * t * t * t;

}

double EvalJerk(const QuinticPolynomial& p, double t) {

    return 6.0 * p.a3 + 24.0 * p.a4 * t + 60.0 * p.a5 * t * t;
}


// ================= Quartic 평가 ========================

double EvalPos(const QuarticPolynomial& p, double t) {

    return p.a0 + p.a1 * t + p.a2 * t * t + p.a3 * t * t * t
        + p.a4 * t * t * t * t;

}

double EvalVel(const QuarticPolynomial& p, double t) {

    return p.a1 + 2.0 * p.a2 * t + 3.0 * p.a3 * t * t 
        + 4.0 * p.a4 * t * t * t;

}

double EvalAcc(const QuarticPolynomial& p, double t) {

    return 2.0 * p.a2 + 6.0 * p.a3 * t + 12.0 * p.a4 * t * t;

}

double EvalJerk(const QuarticPolynomial& p, double t) {

    return 6.0 * p.a3 + 24.0 * p.a4 * t;

}

// ================= JerkCost (닫힌 형태 적분) ==================
//
// Quintic: p'''(t) = 6a3 + 24a4 t + 60a5 t^2
// (p''')^2 = 36a3^2 + 288a3a4 t + (720a3a5+576a4^2) t^2 + 2880a4a5 t^3 + 3600a5^2 t^4
// 0~T 적분:
double JerkCost(const QuinticPolynomial& p, double T) {
    double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;

    return 36.0 * p.a3 * p.a3 * T
         + 144.0 * p.a3 * p.a4 * T2
         + (240.0 * p.a3 * p.a5 + 192.0 * p.a4 * p.a4) * T3
         + 720.0 * p.a4 * p.a5 * T4
         + 720.0 * p.a5 * p.a5 * T5;
}

// Quartic: p'''(t) = 6a3 + 24a4 t
// (p''')^2 = 36a3^2 + 288a3a4 t + 576a4^2 t^2
double JerkCost(const QuarticPolynomial& p, double T) {
    double T2 = T * T, T3 = T2 * T;

    return 36.0 * p.a3 * p.a3 * T
         + 144.0 * p.a3 * p.a4 * T2
         + 192.0 * p.a4 * p.a4 * T3;
}