#pragma once

#include <cstdint>
#include <cstddef>
#include <string>
#include <thread>

// UDP 수신 추상 베이스: bind + 수신 스레드를 담당한다.
// 파생 클래스는 parse_data()만 구현하면 된다 (레퍼런스 receiver.py 대응).
class Receiver {
public:
    Receiver(const std::string& ip, int port);
    virtual ~Receiver();

    Receiver(const Receiver&) = delete;
    Receiver& operator=(const Receiver&) = delete;

protected:
    virtual void parse_data(const uint8_t* raw_data, size_t size) = 0;

private:
    void receive_loop();

    int sock_ = -1;
    std::thread thread_;
    bool running_ = false;
};
