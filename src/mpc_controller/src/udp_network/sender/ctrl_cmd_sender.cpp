#include "udp_network/sender/ctrl_cmd_sender.hpp"

#include <cstdint>
#include <vector>

namespace {
constexpr char kHeader[14] = {'#', 'M', 'o', 'r', 'a', 'i', 'C', 't', 'r', 'l', 'C', 'm', 'd', '$'};
constexpr int32_t kDataLength = 23; // ctrl_mode..steer 페이로드 바이트 수
constexpr char kTail[2] = {'\r', '\n'};

void append(std::vector<uint8_t>& buf, const void* p, size_t n) {
    const uint8_t* b = reinterpret_cast<const uint8_t*>(p);
    buf.insert(buf.end(), b, b + n);
}

}  // namespace

CtrlCmdSender::CtrlCmdSender(const std::string& ip, int port) : Sender(ip, port) {}

void CtrlCmdSender::send(const CtrlCmd& cmd) {
    std::vector<uint8_t> buf;
    buf.reserve(55);

    append(buf, kHeader, sizeof(kHeader));
    append(buf, &kDataLength, sizeof(kDataLength));
    const int32_t aux_data[3] = {0, 0, 0};
    append(buf, aux_data, sizeof(aux_data));
    append(buf, &cmd.ctrl_mode, sizeof(cmd.ctrl_mode));
    append(buf, &cmd.gear, sizeof(cmd.gear));
    append(buf, &cmd.cmd_type, sizeof(cmd.cmd_type));
    append(buf, &cmd.velocity, sizeof(cmd.velocity));
    append(buf, &cmd.acceleration, sizeof(cmd.acceleration));
    append(buf, &cmd.accel, sizeof(cmd.accel));
    append(buf, &cmd.brake, sizeof(cmd.brake));
    append(buf, &cmd.steer, sizeof(cmd.steer));
    append(buf, kTail, sizeof(kTail));

    send_bytes(buf.data(), buf.size());
}
