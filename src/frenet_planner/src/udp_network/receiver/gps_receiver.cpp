#include "udp_network/receiver/gps_receiver.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "udp_network/gps_transform.hpp"

namespace {

constexpr size_t kHeaderSize = 6;
constexpr double kKnotsToMps = 0.514444;

uint64_t MonotonicNowNs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

std::vector<std::string> SplitCsv(const char* data, size_t len) {
    std::vector<std::string> fields;
    size_t start = 0;
    for (size_t i = 0; i <= len; ++i) {
        if (i == len || data[i] == ',' || data[i] == '\0' ||
            data[i] == '\r' || data[i] == '\n') {
            fields.emplace_back(data + start, i - start);
            if (i < len && data[i] == '\0') break;  // 문장 끝(패딩 이전)
            start = i + 1;
        }
    }
    return fields;
}

// NMEA DDMM.MMMM(또는 DDDMM.MMMM) -> 십진 도(decimal degrees). 위경도 필드가
// 비어있거나 숫자가 아니면 false를 반환(수신 도중 fix 없음 등).
bool NmeaCoordToDeg(const std::string& raw, char hemisphere, double& out_deg) {
    if (raw.empty()) return false;
    char* end = nullptr;
    const double value = std::strtod(raw.c_str(), &end);
    if (end == raw.c_str()) return false;

    const double deg_part = std::floor(value / 100.0);
    const double min_part = value - deg_part * 100.0;
    double deg = deg_part + min_part / 60.0;
    if (hemisphere == 'S' || hemisphere == 'W') deg = -deg;
    out_deg = deg;
    return true;
}

}  // namespace

GpsReceiver::GpsReceiver(const std::string& ip, int port) : Receiver(ip, port) {}

void GpsReceiver::update_position(double lat_deg, double lon_deg,
                                  bool has_reported_speed, double reported_speed_mps) {
    double easting, northing;
    GpsToUtm52N(lat_deg, lon_deg, easting, northing);
    const auto now = std::chrono::steady_clock::now();

    std::lock_guard<std::mutex> lock(mutex_);
    double measured_speed = reported_speed_mps;
    // MORAI가 움직이는 중에도 GPRMC speed=0을 보내는 설정이 있으므로 0 값은
    // 정지 근거로 바로 사용하지 않고 위치 변화 기반 정지 판정에 맡긴다.
    bool speed_valid = has_reported_speed && reported_speed_mps >= 0.05;
    bool advance_speed_baseline = !has_previous_position_;
    bool position_changed = !has_previous_position_;
    bool stationary_update = false;
    if (has_previous_position_) {
        const double dt = std::chrono::duration<double>(now - previous_position_time_).count();
        const double distance = std::hypot(easting - previous_easting_,
                                           northing - previous_northing_);
        // MORAI가 같은 GPS fix를 짧은 간격으로 반복 송신할 수 있다. 예전에는
        // 그때마다 기준점을 갱신해 실제 5Hz 위치 변화가 와도 dt가 항상 너무
        // 짧아 속도를 한 번도 계산하지 못했다. 유효 시간간격이 누적됐을 때만
        // 기준점을 전진시킨다.
        constexpr double kPositionChangeThreshold = 0.02;  // [m]
        constexpr double kStationaryConfirmTime = 0.60;    // [s]
        position_changed = distance >= kPositionChangeThreshold;
        if (position_changed) stationary_confirmed_ = false;
        if (position_changed && dt >= 0.10 && dt <= 2.0) {
            const double derived_speed = distance / dt;
            const bool derived_valid = std::isfinite(derived_speed) && derived_speed <= 60.0;
            // 일부 MORAI GPS 설정은 GPRMC speed 필드를 계속 0으로 보낸다.
            // 위치는 실제로 움직였는데 reported speed만 0이면 위치 미분값을 쓴다.
            if (derived_valid &&
                (!has_reported_speed || reported_speed_mps < 0.05 || derived_speed < 0.05)) {
                measured_speed = derived_speed;
                speed_valid = true;
            }
            advance_speed_baseline = true;
        } else if (!position_changed && dt >= kStationaryConfirmTime) {
            // 같은 좌표 한두 패킷은 GPS 5Hz 사이의 반복 전송일 수 있다. 0.6초
            // 이상 실제 위치 변화가 없을 때만 정지로 확정한다.
            measured_speed = 0.0;
            speed_valid = true;
            stationary_confirmed_ = true;
            stationary_update = true;
            advance_speed_baseline = true;
        } else if (dt > 2.0) {
            // 긴 통신 공백 뒤의 위치 점프는 속도로 쓰지 않고 새 기준만 잡는다.
            advance_speed_baseline = true;
        }
    }
    if (speed_valid) {
        // 5Hz GPS 위치 미분 노이즈가 MPC 입력에 바로 들어가지 않도록 저역통과한다.
        // 가속 중 노이즈는 완만하게, 감속/정지 쪽은 빠르게 따라가 MERGE 정지선에서
        // 실제 0인데 추정속도만 1~2m/s 남는 현상을 줄인다.
        const double kSpeedAlpha = measured_speed < fix_.speed_mps ? 0.70 : 0.35;
        fix_.speed_mps = stationary_confirmed_ ? 0.0 : (has_filtered_speed_
            ? (1.0 - kSpeedAlpha) * fix_.speed_mps + kSpeedAlpha * measured_speed
            : measured_speed);
        has_filtered_speed_ = true;
    }
    fix_.lat_deg = lat_deg;
    fix_.lon_deg = lon_deg;
    fix_.has_position = true;
    has_data_ = true;
    fix_.receive_time_ns = MonotonicNowNs();
    fix_.stationary_confirmed = stationary_confirmed_;
    if (position_changed || stationary_update || fix_.position_time_ns == 0)
        fix_.position_time_ns = fix_.receive_time_ns;
    if (advance_speed_baseline) {
        previous_easting_ = easting;
        previous_northing_ = northing;
        previous_position_time_ = now;
        has_previous_position_ = true;
    }
}

void GpsReceiver::parse_data(const uint8_t* raw_data, size_t size) {
    if (size < kHeaderSize) return;

    const std::string header(reinterpret_cast<const char*>(raw_data), kHeaderSize);
    const char* body = reinterpret_cast<const char*>(raw_data) + kHeaderSize;
    const size_t body_len = size - kHeaderSize;

    if (header == "$GPRMC") {
        // split_data 인덱스는 MORAI-NetworkModule lib/define/GPS.py의 GPRMC.parsing과
        // 동일 - body는 "$GPRMC"를 뺀 나머지라 앞에 빈 필드(콤마로 시작) 하나가
        // 먼저 잡히므로 인덱스가 Python의 split_data[1..]과 그대로 맞는다.
        const std::vector<std::string> f = SplitCsv(body, body_len);
        if (f.size() <= 6) return;

        double lat, lon;
        const bool lat_ok = NmeaCoordToDeg(f[3], f[4].empty() ? '\0' : f[4][0], lat);
        const bool lon_ok = NmeaCoordToDeg(f[5], f[6].empty() ? '\0' : f[6][0], lon);
        if (!lat_ok || !lon_ok) return;

        double speed_mps = 0.0;
        bool has_speed = false;
        if (f.size() > 7 && !f[7].empty()) {
            char* end = nullptr;
            const double knots = std::strtod(f[7].c_str(), &end);
            if (end != f[7].c_str()) {
                speed_mps = knots * kKnotsToMps;
                has_speed = true;
            }
        }

        update_position(lat, lon, has_speed, speed_mps);
    } else if (header == "$GPGGA") {
        const std::vector<std::string> f = SplitCsv(body, body_len);
        if (f.size() <= 5) return;

        double lat, lon;
        const bool lat_ok = NmeaCoordToDeg(f[2], f[3].empty() ? '\0' : f[3][0], lat);
        const bool lon_ok = NmeaCoordToDeg(f[4], f[5].empty() ? '\0' : f[5][0], lon);
        if (!lat_ok || !lon_ok) return;

        double alt_m = 0.0;
        bool has_alt = false;
        if (f.size() > 9 && !f[9].empty()) {
            char* end = nullptr;
            const double alt = std::strtod(f[9].c_str(), &end);
            if (end != f[9].c_str()) {
                alt_m = alt;
                has_alt = true;
            }
        }

        update_position(lat, lon, false, 0.0);
        if (has_alt) {
            std::lock_guard<std::mutex> lock(mutex_);
            fix_.alt_m = alt_m;
        }
    }
    // 그 외 문장(GPRMC/GPGGA가 아닌)은 이 프로젝트에서 안 씀 - 조용히 무시.
}

GpsFix GpsReceiver::get_fix() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return fix_;
}

bool GpsReceiver::has_data() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_data_;
}
