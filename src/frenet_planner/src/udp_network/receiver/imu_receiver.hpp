#pragma once

#include <mutex>
#include <string>

#include "udp_network/datatypes.hpp"
#include "udp_network/receiver/receiver.hpp"

// IMU 패킷(network.yaml의 imu_dst_port)을 파싱해 ImuData로 노출한다.
class ImuReceiver : public Receiver {
public:
    ImuReceiver(const std::string& ip, int port);

    ImuData get_data() const;
    bool has_data() const;

protected:
    void parse_data(const uint8_t* raw_data, size_t size) override;

private:
    mutable std::mutex mutex_;
    ImuData data_;
    bool has_data_ = false;
};
