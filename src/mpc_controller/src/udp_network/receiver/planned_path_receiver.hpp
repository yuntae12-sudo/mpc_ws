#pragma once

#include <mutex>
#include <string>

#include "global/global.hpp"
#include "udp_network/receiver/receiver.hpp"

// frenet_planner_node가 보내는 "PlannedPath" 패킷 수신.
// frenet_planner/src/udp_network/sender/planned_path_sender.cpp와 오프셋/필드
// 순서가 반드시 동일해야 한다 (그쪽이 파싱 스펙의 기준).
class PlannedPathReceiver : public Receiver {
public:
    PlannedPathReceiver(const std::string& ip, int port);

    PlannedPath get_path() const;
    bool has_data() const;

protected:
    void parse_data(const uint8_t* raw_data, size_t size) override;

private:
    mutable std::mutex mutex_;
    PlannedPath path_;
    bool has_data_ = false;
};
