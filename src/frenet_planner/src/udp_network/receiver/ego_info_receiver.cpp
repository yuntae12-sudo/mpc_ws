#include "udp_network/receiver/ego_info_receiver.hpp"

#include <cstring>

#include "global/utils.hpp"

namespace {

// EgoVehicleStatus 패킷 레이아웃 (NetworkModule lib/define/EgoVehicleStatus.py 기준, pack=1 가정).
// header(11) + data_length(4) + aux_data(12) = 27 바이트 뒤부터 payload 시작.
// pos_x/pos_y/yaw/속도 오프셋은 실측(MPC의 독립 계산값과 일치)으로 검증 완료.
// TODO: steer_raw는 실측상 rad치고 값이 너무 커서(-10~+6 범위) deg 단위로 의심됨 - 변환 필요성 확인.
// TODO: accel_raw(offset 49)는 실측 속도 변화 추세와 부호가 안 맞아 offset이 틀렸을 가능성 있음.
constexpr size_t kHeaderSize = 11;
constexpr size_t kPayloadOffset = 27;  // header + data_length(int) + aux_data(3 int)
constexpr size_t kMinPacketSize = 229; // payloadOffset..tail 전체 (link_id, tire 계열 포함)

template <typename T>
T read_at(const uint8_t* raw_data, size_t offset) {
    T value;
    std::memcpy(&value, raw_data + offset, sizeof(T));
    return value;
}

}  // namespace

EgoInfoReceiver::EgoInfoReceiver(const std::string& ip, int port) : Receiver(ip, port) {}

void EgoInfoReceiver::parse_data(const uint8_t* raw_data, size_t size) {
    if (size < kMinPacketSize) {
        return;
    }

    const float signed_vel_kmh = read_at<float>(raw_data, kPayloadOffset + 10); // 37
    const float pos_x = read_at<float>(raw_data, kPayloadOffset + 50);          // 77
    const float pos_y = read_at<float>(raw_data, kPayloadOffset + 54);          // 81
    const float yaw_deg = read_at<float>(raw_data, kPayloadOffset + 70);        // 97, roll/pitch/yaw 중 3번째
    const float steer_raw = read_at<float>(raw_data, kPayloadOffset + 110);     // 137
    const float accel_raw = read_at<float>(raw_data, kPayloadOffset + 22);      // 49

    VehicleState state;
    state.x = pos_x;
    state.y = pos_y;
    state.yaw = deg2rad(yaw_deg);
    state.v = signed_vel_kmh / 3.6;
    state.steer = steer_raw;            // TODO: 단위(rad 추정)/부호(좌회전=+) 실측 확인 - deg 단위 의심
    state.accel = accel_raw;            // TODO: offset(49) 실측 속도 변화 추세와 안 맞음, 재확인 필요

    {
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = state;
        has_data_ = true;
    }
}

VehicleState EgoInfoReceiver::get_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

bool EgoInfoReceiver::has_data() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_data_;
}
