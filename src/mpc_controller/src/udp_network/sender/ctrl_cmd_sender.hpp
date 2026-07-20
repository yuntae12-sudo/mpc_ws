#pragma once

#include "udp_network/datatypes.hpp"
#include "udp_network/sender/sender.hpp"

// EgoCtrlCmd 패킷(network.yaml의 ctrl_cmd_host_port) 송신.
// NetworkModule lib/define/EgoCtrlCmd.py 레이아웃 기준 (steer 단일 필드, data_length=23).
class CtrlCmdSender : public Sender {
public:
    CtrlCmdSender(const std::string& ip, int port);

    void send(const CtrlCmd& cmd);
};
