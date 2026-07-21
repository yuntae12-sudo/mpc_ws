#pragma once

#include <memory>
#include <string>
#include <vector>

#include "frenet_planner/global/global.hpp"
#include "udp_network/datatypes.hpp"
#include "udp_network/receiver/ego_info_receiver.hpp"
#include "udp_network/receiver/object_info_receiver.hpp"
#include "udp_network/sender/planned_path_sender.hpp"

// network.yaml 로드 + 소켓 오픈을 담당. main.cpp는 이 클래스를 통해서만
// UDP를 다룬다 (mpc_controller의 UdpManager와 같은 역할 분리 - 알고리즘은
// MORAI/자체 패킷 포맷을 몰라야 한다).
class UdpManager {
public:
    explicit UdpManager(const std::string& config_path);

    bool has_vehicle_state() const;
    VehicleState get_vehicle_state() const;

    bool has_object_info() const;
    std::vector<ObjectInfo> get_objects() const;

    // mpc_node로 계산된 경로 + 중계용 ego 상태 + 진단값(d/d_dot)을 보낸다.
    void send_planned_path(const CartesianPath& cp, double dt, double d, double d_dot,
                            const VehicleState& ego);

private:
    std::unique_ptr<EgoInfoReceiver> ego_info_receiver_;
    std::unique_ptr<ObjectInfoReceiver> object_info_receiver_;
    std::unique_ptr<PlannedPathSender> planned_path_sender_;
};
