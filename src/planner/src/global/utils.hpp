// Planner에서 사용하는 유틸 함수들 모음,, 주석 이용해서 용도 알아보기 용이하게 분류할것.

#ifndef PLANNER_GLOBAL_UTILS_HPP
#define PLANNER_GLOBAL_UTILS_HPP

#include "global/global.hpp"

// ========================================
// 좌표 변환 (mpc_controller/global/utils.hpp와 동일 — path.txt/ref.txt가
// 이 변환으로 만들어졌으므로 planner도 반드시 같은 변환을 써야 RefLine과
// 좌표계가 일치한다).
// ========================================

void wgs84ToECEF(double lat, double lon, double h,
                  double& x, double& y, double& z);

void wgs84ToENU(double lat, double lon, double h,
                 const CoordinateReference& ref,
                 double& x, double& y, double& z);

double quaternionToYaw(double x, double y, double z, double w);

#endif
