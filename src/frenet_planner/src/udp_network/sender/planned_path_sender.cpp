#include "udp_network/sender/planned_path_sender.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>

namespace {

constexpr char kHeader[] = "#FrenetPlanD$";  // 13 chars + null
constexpr size_t kHeaderSize = sizeof(kHeader) - 1;  // null 제외
// 32 * 24B(슬롯) + 71B(고정 헤더/필드) = 839B, UDP fragmentation 임계값(표준
// 이더넷 MTU 1500 - IP/UDP 헤더 = 1472B) 안에 여유 있게 들어간다. 예전에
// 64로 뒀다가(1607B, 1472B 초과) 일부 네트워크 환경에서 조각난 UDP 패킷이
// 조용히 드롭되는 문제가 실측으로 확인됐다 - 실제 count는 보통 20~30개라
// 32면 충분하다.
constexpr size_t kMaxPathSamples = 32;
constexpr char kTail[2] = {'\r', '\n'};

void append(std::vector<uint8_t>& buf, const void* p, size_t n) {
    const uint8_t* b = reinterpret_cast<const uint8_t*>(p);
    buf.insert(buf.end(), b, b + n);
}

}  // namespace

PlannedPathSender::PlannedPathSender(const std::string& ip, int port) : Sender(ip, port) {}

void PlannedPathSender::Send(const CartesianPath& cp, double dt, double d, double d_dot,
                              const VehicleState& ego) {
    const size_t count = std::min(cp.x.size(), kMaxPathSamples);

    std::vector<uint8_t> buf;
    buf.reserve(kHeaderSize + 4 + 12 + 4 + 4 + 4 + 24 + 4 + kMaxPathSamples * 6 * 4 + 2);

    append(buf, kHeader, kHeaderSize);
    const int32_t data_length = 0;  // 고정 레이아웃이라 검증에 안 씀 (기존 관례 유지용 필드)
    append(buf, &data_length, sizeof(data_length));
    const int32_t aux_data[3] = {0, 0, 0};
    append(buf, aux_data, sizeof(aux_data));

    const float dt_f = static_cast<float>(dt);
    const float d_f = static_cast<float>(d);
    const float d_dot_f = static_cast<float>(d_dot);
    append(buf, &dt_f, sizeof(dt_f));
    append(buf, &d_f, sizeof(d_f));
    append(buf, &d_dot_f, sizeof(d_dot_f));

    const float ego_vals[6] = {
        static_cast<float>(ego.x), static_cast<float>(ego.y), static_cast<float>(ego.yaw),
        static_cast<float>(ego.v), static_cast<float>(ego.steer), static_cast<float>(ego.accel)};
    append(buf, ego_vals, sizeof(ego_vals));

    const int32_t count_i32 = static_cast<int32_t>(count);
    append(buf, &count_i32, sizeof(count_i32));

    for (size_t i = 0; i < kMaxPathSamples; ++i) {
        float slot[6] = {0, 0, 0, 0, 0, 0};
        if (i < count) {
            slot[0] = static_cast<float>(cp.x[i]);
            slot[1] = static_cast<float>(cp.y[i]);
            slot[2] = static_cast<float>(cp.yaw[i]);
            slot[3] = static_cast<float>(cp.kappa[i]);
            slot[4] = static_cast<float>(cp.v[i]);
            slot[5] = static_cast<float>(cp.a[i]);
        }
        append(buf, slot, sizeof(slot));
    }

    append(buf, kTail, sizeof(kTail));
    send_bytes(buf.data(), buf.size());
}
