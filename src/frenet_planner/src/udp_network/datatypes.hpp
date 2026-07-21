#pragma once

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
