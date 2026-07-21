#pragma once

#include <string>
#include <vector>

#include "frenet_planner/global/global.hpp"
#include "udp_network/datatypes.hpp"
#include "udp_network/sender/sender.hpp"

// frenet_planner_node -> mpc_node로 보내는 "PlannedPath" 패킷.
// mpc_controller/src/udp_network/receiver/planned_path_receiver.{hpp,cpp}가
// 이 패킷을 파싱하므로 두 파일의 오프셋/필드 순서는 반드시 동일해야 한다
// (이쪽이 파싱 스펙의 기준).
class PlannedPathSender : public Sender {
public:
    PlannedPathSender(const std::string& ip, int port);

    // ego: MORAI에서 받은 값을 그대로 중계(mpc_controller가 더 이상 자체
    // EgoInfoReceiver를 안 쓰므로 여기서 넘겨줘야 함).
    // dt: cp의 샘플 간격 [s] (frenet_planner.sample_dt()).
    // d/d_dot: 진단용 값 (frenet_planner.last_d()/last_d_dot()) - mpc_controller
    // 로그에서 계속 보이게 그대로 전달.
    void Send(const CartesianPath& cp, double dt, double d, double d_dot,
              const VehicleState& ego);
};
