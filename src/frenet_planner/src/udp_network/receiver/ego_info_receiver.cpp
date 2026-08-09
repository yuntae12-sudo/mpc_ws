#include "udp_network/receiver/ego_info_receiver.hpp"

#include <cstring>

#include "global/utils.hpp"

namespace {

// EgoVehicleStatus 패킷 레이아웃 (MORAI-NetworkModule lib/define/EgoVehicleStatus.py 공식
// 정의로 오프셋 재검증 완료, pack=1). header(11) + data_length(4) + aux_data(12) = 27
// 바이트 뒤부터 payload 시작.
// pos_x/pos_y/yaw/속도/steer 오프셋은 공식 정의와 실측(MPC 독립 계산값 일치) 둘 다로 확인됨.
// TODO: steer_raw는 실측상 rad치고 값이 너무 커서(-10~+6 범위) deg 단위로 의심됨 - 변환 필요성 확인
// (오프셋 자체는 공식 정의와 일치하므로 맞음, 단위/부호만 미확인).
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
    // 공식 정의 기준 accel은 kPayloadOffset+18(=45)에 있다. 기존 +22(=49)는
    // 그 다음 필드인 brake를 읽고 있던 오프셋 버그였다(NetworkModule
    // EgoVehicleStatus.py 필드 순서: ..., accel, brake, size_x, ... - accel과
    // brake가 이웃해 있어 한 필드씩 밀려도 값이 그럴듯하게 나와 발견이 늦었음).
    const float accel_raw = read_at<float>(raw_data, kPayloadOffset + 18);      // 45

    VehicleState state;
    state.x = pos_x;
    state.y = pos_y;
    state.yaw = deg2rad(yaw_deg);
    state.v = signed_vel_kmh / 3.6;
    state.steer = steer_raw;            // TODO: 단위(rad 추정)/부호(좌회전=+) 실측 확인 - deg 단위 의심
    state.accel = accel_raw;            // 오프셋 버그 수정 완료(kPayloadOffset+18) - 재검증 필요

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
