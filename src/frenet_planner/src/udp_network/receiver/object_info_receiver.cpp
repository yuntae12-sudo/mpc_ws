#include "udp_network/receiver/object_info_receiver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "global/utils.hpp"

namespace {

constexpr size_t kHeaderSize = 14;
constexpr const char* kExpectedHeader = "#MoraiObjInfo$";
constexpr size_t kDataOffset = 38;
// 공식 ObjectInfo 정의(NetworkModule) 기준 슬롯 stride는 106바이트
// (obj_id~accel_z 68바이트 + link_id[38] 38바이트). 이전에 68로 잘못 잡아서
// 두 번째 객체부터 오프셋이 밀리는 버그가 있었음 - link_id는 안 쓰지만
// stride 계산에는 반드시 포함해야 한다.
constexpr size_t kSlotSize = 106;  // obj_id(2)+objType(2)+pose_xyz(12)+heading(4)+size_xyz(12)
                                   // +overhang(4)+wheelbase(4)+rearoverhang(4)+vel_xyz(12)+accel_xyz(12)
                                   // +link_id(38, 미사용)
constexpr size_t kMaxSlots = 20;
constexpr size_t kMinPacketSize = kDataOffset + kSlotSize;  // 최소 슬롯 1개는 읽을 수 있어야 함
constexpr double kKmhToMps = 1.0 / 3.6;  // Velocity_XYZ 단위가 km/h (문서 확인)

template <typename T>
T read_at(const uint8_t* raw_data, size_t offset) {
    T value;
    std::memcpy(&value, raw_data + offset, sizeof(T));
    return value;
}

}  // namespace

ObjectInfoReceiver::ObjectInfoReceiver(const std::string& ip, int port) : Receiver(ip, port) {}

void ObjectInfoReceiver::parse_data(const uint8_t* raw_data, size_t size) {
    // TEMP DEBUG: 네이티브 MORAI UDP 재검증용 - 브릿지가 아니라 진짜 MORAI가 보낸
    // 패킷인지(2160바이트 근처, 헤더 일치) 확인.
    {
        static int count = 0;
        if (count < 5) {
            ++count;
            const bool header_ok = size >= kHeaderSize &&
                std::memcmp(raw_data, kExpectedHeader, kHeaderSize) == 0;
            std::printf("[ObjectInfo raw] size=%zu header_ok=%d first16=", size, header_ok);
            for (size_t i = 0; i < std::min<size_t>(16, size); ++i) {
                std::printf("%02x ", raw_data[i]);
            }
            std::printf("\n");
        }
    }

    if (size < kMinPacketSize) return;

    const size_t max_slots = std::min(kMaxSlots, (size - kDataOffset) / kSlotSize);

    std::vector<ObjectInfo> objects;
    objects.reserve(max_slots);

    for (size_t i = 0; i < max_slots; ++i) {
        const size_t base = kDataOffset + i * kSlotSize;
        const int16_t obj_id = read_at<int16_t>(raw_data, base + 0);
        if (obj_id == 0) break;  // 빈 슬롯부터는 더 이상 객체 없음 (레퍼런스 스크립트 관례)

        const int16_t obj_type = read_at<int16_t>(raw_data, base + 2);
        const float pose_x = read_at<float>(raw_data, base + 4);
        const float pose_y = read_at<float>(raw_data, base + 8);
        const float heading_deg = read_at<float>(raw_data, base + 16);
        const float size_x = read_at<float>(raw_data, base + 20);  // TODO: 차량 전후(length) 추정
        const float width = read_at<float>(raw_data, base + 24);
        const float vel_x = read_at<float>(raw_data, base + 44);
        const float vel_y = read_at<float>(raw_data, base + 48);

        ObjectInfo obj;
        obj.id = obj_id;
        obj.type = obj_type;
        obj.x = pose_x;
        obj.y = pose_y;
        obj.heading = deg2rad(heading_deg);  // 문서 확인: heading 단위 deg
        obj.speed = std::hypot(vel_x, vel_y) * kKmhToMps;  // 문서 확인: Velocity_XYZ 단위 km/h
        obj.width = width;
        obj.length = size_x;
        obj.yaw_rate = 0.0;  // 수신 패킷에는 yaw-rate가 없어 planner에서 추정
        objects.push_back(obj);
    }

    // TEMP DEBUG: 20Hz 기준 약 1Hz로 파싱된 객체 리스트 출력 - 106-stride로
    // 두 번째 객체 이후도 값이 정상인지 확인.
    {
        static int tick = 0;
        if (++tick % 20 == 0) {
            std::printf("[ObjectInfo parsed] count=%zu\n", objects.size());
            for (const auto& obj : objects) {
                std::printf("  id=%d type=%d pos=(%.3f, %.3f) heading_deg=%.2f "
                            "speed=%.2f m/s size(w=%.2f, l=%.2f)\n",
                            obj.id, obj.type, obj.x, obj.y, rad2deg(obj.heading),
                            obj.speed, obj.width, obj.length);
            }
        }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    objects_ = std::move(objects);
    has_data_ = true;
}

std::vector<ObjectInfo> ObjectInfoReceiver::get_objects() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return objects_;
}

bool ObjectInfoReceiver::has_data() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_data_;
}
