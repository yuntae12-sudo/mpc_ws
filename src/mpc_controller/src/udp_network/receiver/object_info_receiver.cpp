#include "udp_network/receiver/object_info_receiver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "global/utils.hpp"

namespace {

// ObjectInfo 패킷 레이아웃. MORAI 공식 문서(Object Info, 23.R1.0 기준)와 사용자가
// 제공한 실제 파서 스크립트를 근거로 함: header(14, "#MoraiObjInfo$") +
// data_length(4) + aux_data(12) + timestamp(sync 4 + nanosec 4) = 38바이트 뒤부터
// Data 슬롯 20개가 고정으로 이어지고(빈 슬롯은 obj_id==0), 마지막에 tail(2)이 붙는다.
// 슬롯 크기는 처음에 문서의 "106 Bytes*20"(link_id 38바이트 포함)을 그대로 썼다가
// 장애물 인식이 전혀 안 되는 문제가 있었음 - 사용자가 준 실제 동작하는 레퍼런스
// 파서가 link_id를 건너뛰지 않고 68바이트 스트라이드로 다음 객체를 읽는 것으로
// 확인되어(obj_id~accel_z까지만, link_id 제외) 68로 정정.
constexpr size_t kHeaderSize = 14;
constexpr const char* kExpectedHeader = "#MoraiObjInfo$";
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
    // TEMP DEBUG: 소켓에 실제로 데이터가 도착하는지, header 문자열이 예상과
    // 일치하는지부터 확인 (오프셋을 더 추측하기 전에 실측으로 확정하기 위함).
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
