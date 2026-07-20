#pragma once

#include <mutex>
#include <string>

#include "udp_network/datatypes.hpp"
#include "udp_network/receiver/receiver.hpp"

// EgoVehicleStatus 패킷(network.yaml의 ego_vehicle_dst_port)을 파싱해 VehicleState로 노출한다.
class EgoInfoReceiver : public Receiver {
public:
    EgoInfoReceiver(const std::string& ip, int port);

    // 마지막으로 수신한 상태의 복사본. 아직 한 번도 수신 못했으면 has_data()가 false.
    VehicleState get_state() const;
    bool has_data() const;

protected:
    void parse_data(const uint8_t* raw_data, size_t size) override;

private:
    mutable std::mutex mutex_;
    VehicleState state_;
    bool has_data_ = false;
};
