// 임시 진단용 브릿지

#include <algorithm>
#include <arpa/inet.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

#include <ros/ros.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/ObjectStatusList.h>
#include <morai_msgs/ObjectStatus.h>

namespace {

constexpr const char* kDstIp = "127.0.0.1";
constexpr int kObjectDstPort = 7505;
constexpr int kEgoDstPort = 911;

constexpr size_t kHeaderSize = 14;
constexpr const char* kHeader = "#MoraiObjInfo$";
// header(14) + data_length(4) + aux_data(12) + timestamp sync(4) + nanosec(4) = 38
constexpr size_t kDataOffset = 38;
constexpr size_t kSlotSize = 106;  // object_info_receiver.cpp와 동일해야 함
constexpr size_t kMaxSlots = 20;
constexpr size_t kTailSize = 2;

constexpr size_t kEgoPayloadOffset = 27;
constexpr size_t kEgoPacketSize = 229;

template <typename T>
void put_at(std::vector<uint8_t>& buf, size_t offset, T value) {
    std::memcpy(buf.data() + offset, &value, sizeof(T));
}

// morai_msgs/ObjectStatus 하나를 obj_id/objType과 함께 슬롯에 채워 넣는다.
void WriteSlot(std::vector<uint8_t>& buf, size_t slot_index, int16_t obj_id, int16_t obj_type,
               const morai_msgs::ObjectStatus& obj) {
    const size_t base = kDataOffset + slot_index * kSlotSize;

    put_at<int16_t>(buf, base + 0, obj_id);   // 0은 "리스트 끝" sentinel이라 반드시 0이 아니어야 함
    put_at<int16_t>(buf, base + 2, obj_type);
    put_at<float>(buf, base + 4, static_cast<float>(obj.position.x));
    put_at<float>(buf, base + 8, static_cast<float>(obj.position.y));
    put_at<float>(buf, base + 12, static_cast<float>(obj.position.z));
    put_at<float>(buf, base + 16, static_cast<float>(obj.heading));  // deg, MORAI 관례와 동일
    put_at<float>(buf, base + 20, static_cast<float>(obj.size.x));
    put_at<float>(buf, base + 24, static_cast<float>(obj.size.y));
    put_at<float>(buf, base + 28, static_cast<float>(obj.size.z));
    put_at<float>(buf, base + 32, 0.0f);  // overhang (ROS msg에 없음)
    put_at<float>(buf, base + 36, 0.0f);  // wheelbase (ROS msg에 없음)
    put_at<float>(buf, base + 40, 0.0f);  // rearoverhang (ROS msg에 없음)
    // object_info_receiver.cpp가 km/h로 가정하고 3.6으로 나누므로, ROS(m/s)에서 변환.
    put_at<float>(buf, base + 44, static_cast<float>(obj.velocity.x * 3.6));
    put_at<float>(buf, base + 48, static_cast<float>(obj.velocity.y * 3.6));
    put_at<float>(buf, base + 52, static_cast<float>(obj.velocity.z * 3.6));
    put_at<float>(buf, base + 56, static_cast<float>(obj.acceleration.x));
    put_at<float>(buf, base + 60, static_cast<float>(obj.acceleration.y));
    put_at<float>(buf, base + 64, static_cast<float>(obj.acceleration.z));
}

sockaddr_in MakeAddr(int port) {
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    inet_pton(AF_INET, kDstIp, &addr.sin_addr);
    return addr;
}

class Bridge {
public:
    Bridge() {
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        object_addr_ = MakeAddr(kObjectDstPort);
        ego_addr_ = MakeAddr(kEgoDstPort);
    }

    ~Bridge() {
        if (sock_ >= 0) close(sock_);
    }

    void EgoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg) {
        std::vector<uint8_t> buf(kEgoPacketSize, 0);

        const float speed_mps = static_cast<float>(
            std::hypot(msg->velocity.x, msg->velocity.y));
        const float signed_vel_kmh = speed_mps * 3.6f;  // ego_info_receiver.cpp가 /3.6 하므로 역변환
        const float pos_x = static_cast<float>(msg->position.x);
        const float pos_y = static_cast<float>(msg->position.y);
        const float yaw_deg = static_cast<float>(msg->heading);  // ego_info_receiver.cpp가 deg 가정
        const float steer_raw = msg->wheel_angle;
        const float accel_raw = static_cast<float>(msg->acceleration.x);

        std::memcpy(buf.data() + kEgoPayloadOffset + 10, &signed_vel_kmh, sizeof(float));
        std::memcpy(buf.data() + kEgoPayloadOffset + 50, &pos_x, sizeof(float));
        std::memcpy(buf.data() + kEgoPayloadOffset + 54, &pos_y, sizeof(float));
        std::memcpy(buf.data() + kEgoPayloadOffset + 70, &yaw_deg, sizeof(float));
        std::memcpy(buf.data() + kEgoPayloadOffset + 110, &steer_raw, sizeof(float));
        std::memcpy(buf.data() + kEgoPayloadOffset + 22, &accel_raw, sizeof(float));

        sendto(sock_, buf.data(), buf.size(), 0,
               reinterpret_cast<sockaddr*>(&ego_addr_), sizeof(ego_addr_));

        static int log_count = 0;
        if (log_count < 3) {
            ++log_count;
            ROS_INFO("[object_udp_bridge] ego forwarded x=%.2f y=%.2f yaw_deg=%.2f v=%.2f -> %s:%d",
                     pos_x, pos_y, yaw_deg, speed_mps, kDstIp, kEgoDstPort);
        }
    }

    void ObjectCallback(const morai_msgs::ObjectStatusList::ConstPtr& msg) {
        std::vector<const morai_msgs::ObjectStatus*> objects;
        // objType 관례(object_info_receiver.cpp 주석 참고): 보행자=0, 주변차량=1, Object=2
        std::vector<int16_t> types;
        for (const auto& o : msg->pedestrian_list) { objects.push_back(&o); types.push_back(0); }
        for (const auto& o : msg->npc_list)        { objects.push_back(&o); types.push_back(1); }
        for (const auto& o : msg->obstacle_list)   { objects.push_back(&o); types.push_back(2); }

        const size_t n = std::min(objects.size(), kMaxSlots);

        // 항상 kMaxSlots(20)개를 꽉 채워서 보내면 106-stride 기준 2160바이트가
        // 되는데, 이 WSL2(mirrored networking mode)의 127.0.0.1 트래픽은 일반
        // lo(MTU 65536)가 아니라 loopback0(MTU 1500)을 타는 걸로 확인됐다
        // (`ip route get 127.0.0.1`). 2160바이트는 IP 단편화가 필요한데
        // mirrored loopback0에서 단편화된 UDP가 조용히 드롭되는 문제가 실측
        // 확인됨. object_info_receiver.cpp는 obj_id==0을 종료 sentinel로 보고
        // 실제 객체 수만큼만 와도 정상 동작하므로, 굳이 20슬롯을 다 채우지 않고
        // 실제 객체 수(n)만큼만 담아 보낸다 - 대부분 시나리오(장애물 십여 개
        // 이하)에서 1500바이트 미만으로 줄어들어 단편화 자체를 피한다.
        // object_info_receiver.cpp의 kMinPacketSize(=kDataOffset+kSlotSize)를
        // 만족 못 하면 패킷 자체가 무시되어 객체 리스트가 갱신 안 된다(0개로
        // 안 비워짐) - n==0이어도 슬롯 1개 분량은 항상 확보해 obj_id==0
        // sentinel이 제대로 전달되게 한다.
        const size_t slots_to_send = std::max<size_t>(n, 1);
        std::vector<uint8_t> buf(kDataOffset + slots_to_send * kSlotSize + kTailSize, 0);
        std::memcpy(buf.data(), kHeader, kHeaderSize);

        for (size_t i = 0; i < n; ++i) {
            // id 0은 리스트 종료 sentinel이라 항상 1부터 시작하는 인덱스를 씀.
            WriteSlot(buf, i, static_cast<int16_t>(i + 1), types[i], *objects[i]);
        }
        // n개 이후 슬롯은 이미 전부 0으로 채워져 있어(obj_id==0) 자동으로 "끝" 처리됨.

        sendto(sock_, buf.data(), buf.size(), 0,
               reinterpret_cast<sockaddr*>(&object_addr_), sizeof(object_addr_));

        static int log_count = 0;
        if (log_count < 5) {
            ++log_count;
            ROS_INFO("[object_udp_bridge] object forwarded %zu objects (ped=%zu npc=%zu obstacle=%zu) -> %s:%d",
                     n, msg->pedestrian_list.size(), msg->npc_list.size(), msg->obstacle_list.size(),
                     kDstIp, kObjectDstPort);
        }
    }

private:
    int sock_ = -1;
    sockaddr_in object_addr_{};
    sockaddr_in ego_addr_{};
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_udp_bridge_node");
    ros::NodeHandle nh;

    Bridge bridge;
    ros::Subscriber obj_sub = nh.subscribe("/Object_topic", 1, &Bridge::ObjectCallback, &bridge);
    ros::Subscriber ego_sub = nh.subscribe("/Ego_topic", 1, &Bridge::EgoCallback, &bridge);

    ROS_INFO("[object_udp_bridge] /Object_topic -> UDP 127.0.0.1:7505, /Ego_topic -> UDP 127.0.0.1:911 릴레이 시작 (임시 진단용)");
    ros::spin();
    return 0;
}
