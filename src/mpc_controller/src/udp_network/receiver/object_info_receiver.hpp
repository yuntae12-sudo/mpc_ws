#pragma once

#include <mutex>
#include <string>
#include <vector>

#include "frenet_planner/global/global.hpp"
#include "udp_network/receiver/receiver.hpp"

// MORAI ObjectInfo 패킷(network.yaml의 object_info_dst_port)을 파싱해
// std::vector<ObjectInfo>(frenet_planner/global/global.hpp)로 노출한다.
// 알고리즘 계층(FrenetPlanner::Plan)이 그대로 obstacles 인자로 받는 타입이라
// 별도 중립 구조체를 만들지 않고 바로 이 타입으로 변환한다.
class ObjectInfoReceiver : public Receiver {
public:
    ObjectInfoReceiver(const std::string& ip, int port);

    std::vector<ObjectInfo> get_objects() const;
    bool has_data() const;

protected:
    void parse_data(const uint8_t* raw_data, size_t size) override;

private:
    mutable std::mutex mutex_;
    std::vector<ObjectInfo> objects_;
    bool has_data_ = false;
};
