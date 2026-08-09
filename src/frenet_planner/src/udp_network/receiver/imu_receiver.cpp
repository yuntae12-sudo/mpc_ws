#include "udp_network/receiver/imu_receiver.hpp"

#include <cstring>
#include <chrono>

namespace {

uint64_t MonotonicNowNs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

// IMU 패킷 레이아웃 (MORAI-NetworkModule lib/define/IMU.py 기준, pack=1).
// header(9) + data_length(4) + aux_data(4*3=12) + sec(4) + nsec(4) = 33
// 바이트 뒤부터 payload(orientation/각속도/선가속도) 시작.
constexpr size_t kPayloadOffset = 33;
constexpr size_t kMinPacketSize = 115;  // payloadOffset..tail 전체

template <typename T>
T read_at(const uint8_t* raw_data, size_t offset) {
    T value;
    std::memcpy(&value, raw_data + offset, sizeof(T));
    return value;
}

}  // namespace

ImuReceiver::ImuReceiver(const std::string& ip, int port) : Receiver(ip, port) {}

void ImuReceiver::parse_data(const uint8_t* raw_data, size_t size) {
    if (size < kMinPacketSize) return;

    ImuData data;
    data.ori_w = read_at<double>(raw_data, kPayloadOffset + 0);
    data.ori_x = read_at<double>(raw_data, kPayloadOffset + 8);
    data.ori_y = read_at<double>(raw_data, kPayloadOffset + 16);
    data.ori_z = read_at<double>(raw_data, kPayloadOffset + 24);
    data.ang_vel_x = read_at<double>(raw_data, kPayloadOffset + 32);
    data.ang_vel_y = read_at<double>(raw_data, kPayloadOffset + 40);
    data.ang_vel_z = read_at<double>(raw_data, kPayloadOffset + 48);
    data.lin_acc_x = read_at<double>(raw_data, kPayloadOffset + 56);
    data.lin_acc_y = read_at<double>(raw_data, kPayloadOffset + 64);
    data.lin_acc_z = read_at<double>(raw_data, kPayloadOffset + 72);
    data.receive_time_ns = MonotonicNowNs();

    std::lock_guard<std::mutex> lock(mutex_);
    data_ = data;
    has_data_ = true;
}

ImuData ImuReceiver::get_data() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return data_;
}

bool ImuReceiver::has_data() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_data_;
}
