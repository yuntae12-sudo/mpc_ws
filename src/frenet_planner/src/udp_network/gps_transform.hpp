#pragma once

// GPS(WGS84)/IMU 원시값 -> 이 프로젝트의 로컬 평면 좌표(x=East, y=North)/yaw로
// 변환하는 순수 함수. ROS 의존성 없음(Study_ITS의 Global.cpp gps2Enu/yawTf를
// UDP 파이프라인에 맞게 포팅) - GPS/IMU 리시버 어느 쪽에도 묶이지 않고
// UdpManager가 최신 GpsFix+ImuData를 조합할 때 호출한다.

// WGS84 -> ECEF -> 기준점(ref_lat/ref_lon/ref_alt) 기준 ENU(East-North-Up) 변환.
// 원본(Study_ITS Global.cpp)은 Up 성분만 기준점이 아니라 현재 위치의 위/경도로
// 회전행렬을 만드는 버그가 있었다(East/North는 기준점 기준인데 Up만 다른
// 기준을 씀) - 세 축 모두 기준점 기준으로 통일해서 포팅한다. up은 이
// 프로젝트가 2D(x,y,yaw)만 쓰므로 호출부에서 버려도 무방.
void GpsToEnu(double lat_deg, double lon_deg, double alt_m,
              double ref_lat_deg, double ref_lon_deg, double ref_alt_m,
              double& out_e, double& out_n, double& out_u);

// WGS84 위경도를 UTM Zone 52N(EPSG:32652) Easting/Northing으로 변환한다.
// MORAI global path는 이 좌표에서 GPS Info의 East/North Offset을 뺀 좌표다.
void GpsToUtm52N(double lat_deg, double lon_deg,
                 double& out_easting, double& out_northing);

// IMU 쿼터니안(w,x,y,z) -> yaw [rad], 범위 (-pi, pi].
double QuaternionToYaw(double w, double x, double y, double z);
