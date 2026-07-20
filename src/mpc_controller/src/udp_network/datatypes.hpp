#pragma once

#include <cstdint>


// 통신(UDP) <-> 알고리즘 경계의 중립 데이터 구조체.
// 알고리즘 계층은 이 구조체만 알고, MORAI 원시 패킷 레이아웃은 receiver/*.cpp 안에만 존재해야 한다.

// EgoVehicleStatus 수신 패킷(ego_vehicle_dst_port)에서 뽑아낸 값.
struct VehicleState {
    double x = 0.0;      // pos_x [m]
    double y = 0.0;      // pos_y [m]
    double yaw = 0.0;    // [rad] -- TODO: EgoVehicleStatus.yaw 원본 단위(deg 추정) 실측 로그로 확인 후 변환 확정
    double v = 0.0;      // [m/s] -- TODO: signed_vel vs vel_x 중 어느 필드를 쓸지, 단위(km/h?) 확인 필요
    double steer = 0.0;  // [rad] -- EgoVehicleStatus.steer, 부호(좌회전=+) 실측 확인 필요
    double accel = 0.0;  // [m/s^2] EgoVehicleStatus.accel passthrough
};

// EgoCtrlCmd 송신 페이로드(ctrl_cmd_host_port). steer 단일 필드, data_length=23 버전.
struct CtrlCmd {
    int8_t ctrl_mode = 2;      // 1: Keyboard, 2: AutoMode
    int8_t gear = 4;           // 0:M 1:P 2:R 3:N 4:D 5:L
    int8_t cmd_type = 1;       // 1: Throttle(accel,brake,steer) 2: Velocity 3: Acceleration
    float velocity = 0.0f;     // cmd_type == 2 일 때 사용 [km/h]
    float acceleration = 0.0f; // cmd_type == 3 일 때 사용 [m/s^2]
    float accel = 0.0f;        // [0, 1]
    float brake = 0.0f;        // [0, 1]
    float steer = 0.0f;        // [-1, 1]
};

