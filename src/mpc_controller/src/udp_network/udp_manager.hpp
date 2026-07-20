#pragma once

#include <memory>
#include <string>
#include <vector>

#include "frenet_planner/global/global.hpp"
#include "udp_network/datatypes.hpp"
#include "udp_network/receiver/ego_info_receiver.hpp"
#include "udp_network/receiver/object_info_receiver.hpp"
#include "udp_network/sender/ctrl_cmd_sender.hpp"

// network.yaml 로드 + 소켓 오픈을 담당. main.cpp/알고리즘 계층은 이 클래스를 통해서만
// UDP를 다룬다 (알고리즘은 MORAI 패킷 포맷을 몰라야 한다).
class UdpManager {
public:
    explicit UdpManager(const std::string& config_path);

    bool has_vehicle_state() const;
    VehicleState get_vehicle_state() const;

    // has_data()가 false여도 get_objects()는 빈 리스트를 반환한다(장애물 없음과
    // 동일하게 다뤄도 안전 - 아직 한 번도 패킷을 못 받은 것과 "장애물 0개"인
    // 실제 상황을 호출부에서 구분할 필요가 없는 obstacles 용도이기 때문).
    bool has_object_info() const;
    std::vector<ObjectInfo> get_objects() const;

    void send_ctrl_cmd(const CtrlCmd& cmd);

private:
    std::unique_ptr<EgoInfoReceiver> ego_info_receiver_;
    std::unique_ptr<ObjectInfoReceiver> object_info_receiver_;
    std::unique_ptr<CtrlCmdSender> ctrl_cmd_sender_;
};
