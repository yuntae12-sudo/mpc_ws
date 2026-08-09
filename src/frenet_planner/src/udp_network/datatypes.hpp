#pragma once

#include <cstdint>

// 통신(UDP) <-> 알고리즘 경계의 중립 데이터 구조체.
// mpc_controller/src/udp_network/datatypes.hpp에서 이 패키지가 실제로 쓰는
// VehicleState만 가져옴 - CtrlCmd(MORAI로 보내는 제어 출력)는 mpc_controller
// 담당이라 여기서는 필요 없다.

// EgoVehicleStatus 수신 패킷(ego_vehicle_dst_port)에서 뽑아낸 값.
struct VehicleState {
    double x = 0.0;      // pos_x [m]
    double y = 0.0;      // pos_y [m]
    double yaw = 0.0;    // [rad] -- TODO: EgoVehicleStatus.yaw 원본 단위(deg 추정) 실측 로그로 확인 후 변환 확정
    double v = 0.0;      // [m/s] -- TODO: signed_vel vs vel_x 중 어느 필드를 쓸지, 단위(km/h?) 확인 필요
    double steer = 0.0;  // [rad] -- EgoVehicleStatus.steer, 부호(좌회전=+) 실측 확인 필요
    double accel = 0.0;  // [m/s^2] EgoVehicleStatus.accel passthrough
};

// GPS 수신 패킷(gps_dst_port)에서 뽑아낸 최신 fix. NMEA GPGGA/GPRMC는 서로 다른
// 필드를 담고 있어(GPGGA: 위치+고도+fix quality, GPRMC: 위치+속도+방위) 둘 다
// 최신값을 유지하며 병합한다 - gps_receiver.cpp 주석 참고.
struct GpsFix {
    double lat_deg = 0.0;      // GPGGA/GPRMC 공통 [deg], 최신 수신한 문장 기준
    double lon_deg = 0.0;      // [deg]
    double alt_m = 0.0;        // GPGGA 전용 [m], GPRMC만 온 경우 이전 값 유지
    double speed_mps = 0.0;    // GPRMC 전용 [m/s] (knots -> m/s 변환됨), GPGGA만 온 경우 이전 값 유지
    bool has_position = false;
    uint64_t receive_time_ns = 0;   // 마지막 유효 GPS 패킷 수신 monotonic time
    uint64_t position_time_ns = 0;  // 마지막 실제 위치 변화 fix 시각
    bool stationary_confirmed = false;
};

// IMU 수신 패킷(imu_dst_port)에서 뽑아낸 최신 상태. orientation은 단위 쿼터니안.
struct ImuData {
    double ori_w = 1.0, ori_x = 0.0, ori_y = 0.0, ori_z = 0.0;
    double ang_vel_x = 0.0, ang_vel_y = 0.0, ang_vel_z = 0.0;  // [rad/s]
    double lin_acc_x = 0.0, lin_acc_y = 0.0, lin_acc_z = 0.0;  // [m/s^2]
    uint64_t receive_time_ns = 0;  // 마지막 유효 IMU 패킷 수신 monotonic time
};
