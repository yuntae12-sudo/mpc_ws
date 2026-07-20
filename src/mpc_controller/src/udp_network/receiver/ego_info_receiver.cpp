#include "udp_network/receiver/ego_info_receiver.hpp"

#include <cstring>

#include "global/utils.hpp"

namespace {

// EgoVehicleStatus 패킷 레이아웃 (NetworkModule lib/define/EgoVehicleStatus.py 기준, pack=1 가정).
// header(11) + data_length(4) + aux_data(12) = 27 바이트 뒤부터 payload 시작.
// TODO: 아래 오프셋/필드 존재 여부는 실제 UDP 로그 캡처로 검증 전까지는 추정치다.
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

    // header 문자열은 검증하지 않는다 (EgoVehicleStatus.py에 기본 header 텍스트가 없음).
    (void)kHeaderSize;

    const float signed_vel_kmh = read_at<float>(raw_data, kPayloadOffset + 10); // 37 (km/h로 추정)
    const float pos_x = read_at<float>(raw_data, kPayloadOffset + 50);          // 77
    const float pos_y = read_at<float>(raw_data, kPayloadOffset + 54);          // 81
    const float yaw_deg = read_at<float>(raw_data, kPayloadOffset + 70);        // 97, roll/pitch/yaw 중 3번째
    const float steer_raw = read_at<float>(raw_data, kPayloadOffset + 110);     // 137
    const float accel_raw = read_at<float>(raw_data, kPayloadOffset + 22);      // 49

    VehicleState state;
    state.x = pos_x;
    state.y = pos_y;
    state.yaw = deg2rad(yaw_deg);       // TODO: 실측으로 deg 가정 확인
    state.v = signed_vel_kmh / 3.6;     // TODO: signed_vel vs vel_x 중 선택 확정, 부호 확인
    state.steer = steer_raw;            // TODO: 단위(rad 추정)/부호(좌회전=+) 실측 확인
    state.accel = accel_raw;

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
