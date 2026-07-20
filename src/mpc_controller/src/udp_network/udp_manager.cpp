#include "udp_network/udp_manager.hpp"

#include <cstdio>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

UdpManager::UdpManager(const std::string& config_path) {
    YAML::Node root = YAML::LoadFile(config_path);
    YAML::Node network = root["network"];

    const std::string host_ip = network["host_ip"].as<std::string>();
    const std::string user_ip = network["user_ip"].as<std::string>();
    const int ego_vehicle_dst_port = network["ego_vehicle_dst_port"].as<int>();
    const int ctrl_cmd_host_port = network["ctrl_cmd_host_port"].as<int>();
    const int object_info_dst_port = network["object_info_dst_port"].as<int>();

    std::printf("[UDP] ego_vehicle recv <- %s:%d\n", host_ip.c_str(), ego_vehicle_dst_port);
    std::printf("[UDP] object_info recv <- %s:%d\n", host_ip.c_str(), object_info_dst_port);
    std::printf("[UDP] ctrl_cmd send  -> %s:%d\n", user_ip.c_str(), ctrl_cmd_host_port);

    ego_info_receiver_ = std::make_unique<EgoInfoReceiver>(host_ip, ego_vehicle_dst_port);
    object_info_receiver_ = std::make_unique<ObjectInfoReceiver>(host_ip, object_info_dst_port);
    ctrl_cmd_sender_ = std::make_unique<CtrlCmdSender>(user_ip, ctrl_cmd_host_port);
}

bool UdpManager::has_vehicle_state() const {
    return ego_info_receiver_->has_data();
}

VehicleState UdpManager::get_vehicle_state() const {
    return ego_info_receiver_->get_state();
}

bool UdpManager::has_object_info() const {
    return object_info_receiver_->has_data();
}

std::vector<ObjectInfo> UdpManager::get_objects() const {
    return object_info_receiver_->get_objects();
}

void UdpManager::send_ctrl_cmd(const CtrlCmd& cmd) {
    ctrl_cmd_sender_->send(cmd);
}
