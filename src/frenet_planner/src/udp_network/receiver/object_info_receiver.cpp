#include "udp_network/receiver/object_info_receiver.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "global/utils.hpp"

namespace {

constexpr size_t kDataOffset = 38;
constexpr size_t kSlotSize = 68;  // obj_id(2)+objType(2)+pose_xyz(12)+heading(4)+size_xyz(12)
                                  // +overhang(4)+wheelbase(4)+rearoverhang(4)+vel_xyz(12)+accel_xyz(12)
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
        const float size_y = read_at<float>(raw_data, base + 24);  // TODO: 차량 좌우(width) 추정
        const float vel_x = read_at<float>(raw_data, base + 44);
        const float vel_y = read_at<float>(raw_data, base + 48);

        ObjectInfo obj;
        obj.id = obj_id;
        obj.type = obj_type;
        obj.x = pose_x;
        obj.y = pose_y;
        obj.heading = deg2rad(heading_deg);  // 문서 확인: heading 단위 deg
        obj.speed = std::hypot(vel_x, vel_y) * kKmhToMps;  // 문서 확인: Velocity_XYZ 단위 km/h
        obj.width = size_y;
        obj.length = size_x;
        objects.push_back(obj);
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
