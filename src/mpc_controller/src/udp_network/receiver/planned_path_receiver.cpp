#include "udp_network/receiver/planned_path_receiver.hpp"

#include <cstdint>
#include <cstring>

namespace {

// frenet_planner/src/udp_network/sender/planned_path_sender.cpp와 반드시 동일해야
// 하는 레이아웃. header(13) + data_length(4) + aux_data(12) = 29바이트 뒤부터
// payload 시작.
constexpr size_t kHeaderSize = 13;
constexpr char kExpectedHeader[] = "#FrenetPlanD$";
constexpr size_t kPayloadOffset = 29;  // header + data_length(int32) + aux_data(3*int32)
// planned_path_sender.cpp의 kMaxPathSamples와 반드시 같은 값이어야 한다
// (1472B UDP fragmentation 임계값 이하로 유지하는 이유는 그쪽 주석 참고).
constexpr size_t kMaxPathSamples = 32;
constexpr size_t kSlotSize = 6 * sizeof(float);  // x,y,yaw,kappa,v,a
constexpr size_t kSamplesOffset = kPayloadOffset + 4 + 4 + 4 + 24 + 4;  // dt,d,d_dot,ego(6),count 다음
constexpr size_t kMinPacketSize = kSamplesOffset + kMaxPathSamples * kSlotSize;

template <typename T>
T read_at(const uint8_t* raw_data, size_t offset) {
    T value;
    std::memcpy(&value, raw_data + offset, sizeof(T));
    return value;
}

}  // namespace

PlannedPathReceiver::PlannedPathReceiver(const std::string& ip, int port) : Receiver(ip, port) {}

void PlannedPathReceiver::parse_data(const uint8_t* raw_data, size_t size) {
    if (size < kMinPacketSize) return;
    if (std::memcmp(raw_data, kExpectedHeader, kHeaderSize) != 0) return;

    PlannedPath path;
    path.dt    = read_at<float>(raw_data, kPayloadOffset + 0);
    path.d     = read_at<float>(raw_data, kPayloadOffset + 4);
    path.d_dot = read_at<float>(raw_data, kPayloadOffset + 8);

    path.ego_x     = read_at<float>(raw_data, kPayloadOffset + 12);
    path.ego_y     = read_at<float>(raw_data, kPayloadOffset + 16);
    path.ego_yaw   = read_at<float>(raw_data, kPayloadOffset + 20);
    path.ego_v     = read_at<float>(raw_data, kPayloadOffset + 24);
    path.ego_steer = read_at<float>(raw_data, kPayloadOffset + 28);
    path.ego_accel = read_at<float>(raw_data, kPayloadOffset + 32);

    const int32_t count = read_at<int32_t>(raw_data, kPayloadOffset + 36);
    const size_t n = (count > 0 && static_cast<size_t>(count) <= kMaxPathSamples)
                         ? static_cast<size_t>(count)
                         : 0;

    path.x.reserve(n); path.y.reserve(n); path.yaw.reserve(n);
    path.kappa.reserve(n); path.v.reserve(n); path.a.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        const size_t base = kSamplesOffset + i * kSlotSize;
        path.x.push_back(read_at<float>(raw_data, base + 0));
        path.y.push_back(read_at<float>(raw_data, base + 4));
        path.yaw.push_back(read_at<float>(raw_data, base + 8));
        path.kappa.push_back(read_at<float>(raw_data, base + 12));
        path.v.push_back(read_at<float>(raw_data, base + 16));
        path.a.push_back(read_at<float>(raw_data, base + 20));
    }

    std::lock_guard<std::mutex> lock(mutex_);
    path_ = std::move(path);
    has_data_ = true;
}

PlannedPath PlannedPathReceiver::get_path() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return path_;
}

bool PlannedPathReceiver::has_data() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_data_;
}
