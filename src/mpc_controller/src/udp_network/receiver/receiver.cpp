#include "udp_network/receiver/receiver.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <stdexcept>
#include <vector>

namespace {
constexpr size_t kRecvBufferSize = 65535;
}

Receiver::Receiver(const std::string& ip, int port) {
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
        throw std::runtime_error("Receiver: socket() failed");
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    addr.sin_addr.s_addr = inet_addr(ip.c_str());

    if (bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(sock_);
        throw std::runtime_error("Receiver: bind() failed on " + ip + ":" + std::to_string(port));
    }

    running_ = true;
    thread_ = std::thread(&Receiver::receive_loop, this);
}

Receiver::~Receiver() {
    running_ = false;
    if (sock_ >= 0) {
        // 블로킹 중인 recv()를 깨우기 위해 소켓을 먼저 닫는다.
        close(sock_);
    }
    if (thread_.joinable()) {
        thread_.join();
    }
}

void Receiver::receive_loop() {
    std::vector<uint8_t> buffer(kRecvBufferSize);
    while (running_) {
        ssize_t n = recv(sock_, buffer.data(), buffer.size(), 0);
        if (n > 0) {
            parse_data(buffer.data(), static_cast<size_t>(n));
        }
    }
}
