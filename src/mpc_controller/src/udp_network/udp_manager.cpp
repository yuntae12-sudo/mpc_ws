#include "udp_network/udp_manager.hpp"

#include <cstdio>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

UdpManager::UdpManager(const std::string& config_path) {
    YAML::Node root = YAML::LoadFile(config_path);
    YAML::Node network = root["network"];

    const std::string host_ip = network["host_ip"].as<std::string>();
    const std::string user_ip = network["user_ip"].as<std::string>();
    const int planned_path_host_port = network["planned_path_host_port"].as<int>();
    const int ctrl_cmd_host_port = network["ctrl_cmd_host_port"].as<int>();

    std::printf("[UDP] planned_path recv <- %s:%d\n", host_ip.c_str(), planned_path_host_port);
    std::printf("[UDP] ctrl_cmd send     -> %s:%d\n", user_ip.c_str(), ctrl_cmd_host_port);

    planned_path_receiver_ = std::make_unique<PlannedPathReceiver>(host_ip, planned_path_host_port);
    ctrl_cmd_sender_ = std::make_unique<CtrlCmdSender>(user_ip, ctrl_cmd_host_port);
}

bool UdpManager::has_planned_path() const {
    return planned_path_receiver_->has_data();
}

PlannedPath UdpManager::get_planned_path() const {
    return planned_path_receiver_->get_path();
}

void UdpManager::send_ctrl_cmd(const CtrlCmd& cmd) {
    ctrl_cmd_sender_->send(cmd);
}
