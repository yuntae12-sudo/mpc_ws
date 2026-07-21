#include "udp_network/udp_manager.hpp"

#include <cstdio>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

UdpManager::UdpManager(const std::string& config_path) {
    YAML::Node root = YAML::LoadFile(config_path);
    YAML::Node network = root["network"];

    const std::string host_ip = network["host_ip"].as<std::string>();
    const std::string mpc_ip = network["mpc_ip"].as<std::string>();
    const int ego_vehicle_dst_port = network["ego_vehicle_dst_port"].as<int>();
    const int object_info_dst_port = network["object_info_dst_port"].as<int>();
    const int planned_path_dst_port = network["planned_path_dst_port"].as<int>();

    std::printf("[UDP] ego_vehicle recv    <- %s:%d\n", host_ip.c_str(), ego_vehicle_dst_port);
    std::printf("[UDP] object_info recv    <- %s:%d\n", host_ip.c_str(), object_info_dst_port);
    std::printf("[UDP] planned_path send   -> %s:%d\n", mpc_ip.c_str(), planned_path_dst_port);

    ego_info_receiver_ = std::make_unique<EgoInfoReceiver>(host_ip, ego_vehicle_dst_port);
    object_info_receiver_ = std::make_unique<ObjectInfoReceiver>(host_ip, object_info_dst_port);
    planned_path_sender_ = std::make_unique<PlannedPathSender>(mpc_ip, planned_path_dst_port);
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

void UdpManager::send_planned_path(const CartesianPath& cp, double dt, double d, double d_dot,
                                    const VehicleState& ego) {
    planned_path_sender_->Send(cp, dt, d, d_dot, ego);
}
