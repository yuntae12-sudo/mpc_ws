#include "udp_network/sender/sender.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <stdexcept>

Sender::Sender(const std::string& ip, int port) : ip_(ip), port_(port) {
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
        throw std::runtime_error("Sender: socket() failed");
    }
}

Sender::~Sender() {
    if (sock_ >= 0) {
        close(sock_);
    }
}

void Sender::send_bytes(const uint8_t* data, size_t size) {
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port_));
    addr.sin_addr.s_addr = inet_addr(ip_.c_str());

    sendto(sock_, data, size, 0, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
}
