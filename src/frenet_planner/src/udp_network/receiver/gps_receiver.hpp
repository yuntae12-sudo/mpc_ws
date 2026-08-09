#pragma once

#include <chrono>
#include <mutex>
#include <string>

#include "udp_network/datatypes.hpp"
#include "udp_network/receiver/receiver.hpp"

// GPS 패킷(network.yaml의 gps_dst_port)을 파싱해 GpsFix로 노출한다.
// MORAI-NetworkModule lib/define/GPS.py 기준: 바이너리가 아니라 NMEA 0183
// 문장(GPRMC/GPGGA)을 그대로 UDP에 실은 것 - header[6]("$GPRMC"/"$GPGGA")
// 뒤에 나머지 문장이 comma-separated 텍스트로 이어진다.
class GpsReceiver : public Receiver {
public:
    GpsReceiver(const std::string& ip, int port);

    // 마지막으로 수신한 fix의 복사본. 아직 위치를 한 번도 못 받았으면
    // has_data()가 false.
    GpsFix get_fix() const;
    bool has_data() const;

protected:
    void parse_data(const uint8_t* raw_data, size_t size) override;

private:
    void update_position(double lat_deg, double lon_deg,
                         bool has_reported_speed, double reported_speed_mps);

    mutable std::mutex mutex_;
    GpsFix fix_;
    bool has_data_ = false;
    bool has_previous_position_ = false;
    double previous_easting_ = 0.0;
    double previous_northing_ = 0.0;
    std::chrono::steady_clock::time_point previous_position_time_{};
    bool has_filtered_speed_ = false;
    bool stationary_confirmed_ = false;
};
