#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

// UDP 송신 추상 베이스. 고정된 목적지(ip:port)로 sendto한다.
class Sender {
public:
    Sender(const std::string& ip, int port);
    virtual ~Sender();

    Sender(const Sender&) = delete;
    Sender& operator=(const Sender&) = delete;

protected:
    void send_bytes(const uint8_t* data, size_t size);

private:
    int sock_ = -1;
    std::string ip_;
    int port_;
};
