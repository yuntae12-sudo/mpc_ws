#pragma once

#include <memory>
#include <string>

#include "global/global.hpp"
#include "udp_network/datatypes.hpp"
#include "udp_network/receiver/planned_path_receiver.hpp"
#include "udp_network/sender/ctrl_cmd_sender.hpp"

// network.yaml 로드 + 소켓 오픈을 담당. main.cpp/알고리즘 계층은 이 클래스를 통해서만
// UDP를 다룬다 (알고리즘은 MORAI/frenet_planner 패킷 포맷을 몰라야 한다).
//
// frenet_planner 패키지 분리 이후: mpc_controller는 더 이상 MORAI의 ego/object
// 포트를 직접 듣지 않는다 - frenet_planner_node가 그걸 대신 받아서 계산한 경로와
// 중계용 ego 상태를 PlannedPath 패킷 하나로 묶어 보내주고, 여기서는 그것만 받는다.
class UdpManager {
public:
    explicit UdpManager(const std::string& config_path);

    bool has_planned_path() const;
    PlannedPath get_planned_path() const;

    void send_ctrl_cmd(const CtrlCmd& cmd);

private:
    std::unique_ptr<PlannedPathReceiver> planned_path_receiver_;
    std::unique_ptr<CtrlCmdSender> ctrl_cmd_sender_;
};
