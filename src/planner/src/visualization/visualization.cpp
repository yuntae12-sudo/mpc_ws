#include "visualization/visualization.hpp"

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

namespace {

// yaw만 있는 회전을 쿼터니언으로 (tf2 의존성 추가하지 않기 위해 직접 계산).
geometry_msgs::Quaternion YawToQuaternion(double yaw) {
    geometry_msgs::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw / 2.0);
    q.w = std::cos(yaw / 2.0);
    return q;
}

geometry_msgs::Point MakePoint(double x, double y, double z = 0.0) {
    geometry_msgs::Point p;
    p.x = x; p.y = y; p.z = z;
    return p;
}

std_msgs::ColorRGBA MakeColor(float r, float g, float b, float a) {
    std_msgs::ColorRGBA c;
    c.r = r; c.g = g; c.b = b; c.a = a;
    return c;
}

}  // namespace

visualization_msgs::MarkerArray BuildCandidateMarkers(const std::vector<FrenetPath>& candidates,
                                                       const RefLine& ref,
                                                       const FrenetPath* best,
                                                       const std::string& frame_id) {
    visualization_msgs::MarkerArray array;
    const ros::Time stamp = ros::Time::now();
    const ros::Duration lifetime(0.2);  // 다음 사이클(보통 100ms) 전에 갱신 안 되면 자동 소멸

    int id = 0;
    for (const auto& path : candidates) {
        // best는 따로 맨 나중에 그려서 항상 다른 후보 위에 덮어 보이게 한다.
        if (&path == best) continue;

        visualization_msgs::Marker m;
        m.header.frame_id = frame_id;
        m.header.stamp = stamp;
        m.ns = "candidates";
        m.id = id++;
        m.type = visualization_msgs::Marker::LINE_STRIP;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.lifetime = lifetime;

        // 논문 Fig.3~6: valid=회색(장기목표 대안), invalid=옅은 빨강.
        if (path.valid) {
            m.scale.x = 0.05;
            m.color = MakeColor(0.6f, 0.6f, 0.6f, 0.5f);
        } else {
            m.scale.x = 0.03;
            m.color = MakeColor(0.9f, 0.2f, 0.2f, 0.25f);
        }

        CartesianPath cp = ConvertToCartesianPath(path, ref);
        m.points.reserve(cp.x.size());
        for (size_t i = 0; i < cp.x.size(); i++) {
            m.points.push_back(MakePoint(cp.x[i], cp.y[i]));
        }
        array.markers.push_back(std::move(m));
    }

    if (best != nullptr) {
        visualization_msgs::Marker m;
        m.header.frame_id = frame_id;
        m.header.stamp = stamp;
        m.ns = "candidates";
        m.id = id++;  // 다른 ns:"best"로 분리해도 되지만, 같은 배열 안 id 충돌만 피하면 됨
        m.type = visualization_msgs::Marker::LINE_STRIP;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.lifetime = lifetime;
        m.scale.x = 0.15;
        m.color = MakeColor(0.1f, 0.9f, 0.1f, 1.0f);

        CartesianPath cp = ConvertToCartesianPath(*best, ref);
        m.points.reserve(cp.x.size());
        for (size_t i = 0; i < cp.x.size(); i++) {
            m.points.push_back(MakePoint(cp.x[i], cp.y[i]));
        }
        array.markers.push_back(std::move(m));
    }

    return array;
}

visualization_msgs::Marker BuildRefLineMarker(const RefLine& ref, double center_s, double window,
                                               const std::string& frame_id, int id) {
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = ros::Time::now();
    m.ns = "refline";
    m.id = id;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.lifetime = ros::Duration(0.2);
    m.scale.x = 0.08;
    m.color = MakeColor(0.2f, 0.4f, 1.0f, 0.8f);

    const double s_min = std::max(0.0, center_s - window);
    const double s_max = center_s + window;
    constexpr double kStep = 0.5;  // 시각화용 재샘플 간격 (RefLine 자체는 0.1m라 더 촘촘함 — 낭비 방지)
    for (double s = s_min; s <= s_max; s += kStep) {
        RefPoint rp = Interpolate(ref, s);
        m.points.push_back(MakePoint(rp.x, rp.y));
    }
    return m;
}

visualization_msgs::Marker BuildGlobalPathMarker(const RefLine& ref, const std::string& frame_id, int id) {
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = ros::Time::now();
    m.ns = "global_path";
    m.id = id;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.lifetime = ros::Duration(0.0);  // 0 = 영구 (latched 토픽으로 한 번만 발행할 것이므로)
    m.scale.x = 0.15;
    m.color = MakeColor(1.0f, 1.0f, 1.0f, 0.6f);

    const double total_s = ref.points.back().s;
    constexpr double kStep = 1.0;  // 전체 조망용이라 근방 마커(0.5m)보다 더 성기게 샘플링
    m.points.reserve(static_cast<size_t>(total_s / kStep) + 2);
    for (double s = 0.0; s <= total_s; s += kStep) {
        RefPoint rp = Interpolate(ref, s);
        m.points.push_back(MakePoint(rp.x, rp.y));
    }
    return m;
}

visualization_msgs::Marker BuildEgoMarker(const CartesianState& cs, const VehicleShape& shape,
                                           const std::string& frame_id, int id) {
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = ros::Time::now();
    m.ns = "ego";
    m.id = id;
    m.type = visualization_msgs::Marker::CUBE;
    m.action = visualization_msgs::Marker::ADD;
    m.lifetime = ros::Duration(0.2);

    m.pose.position = MakePoint(cs.x, cs.y, shape.width * 0.3);  // 바닥에서 살짝 띄워서 잘 보이게
    m.pose.orientation = YawToQuaternion(cs.yaw);
    m.scale.x = shape.length;
    m.scale.y = shape.width;
    m.scale.z = shape.width * 0.6;
    m.color = MakeColor(1.0f, 0.9f, 0.1f, 0.9f);

    return m;
}

visualization_msgs::MarkerArray BuildObstacleMarkers(const std::vector<ObjectInfo>& obstacles,
                                                      const std::string& frame_id) {
    visualization_msgs::MarkerArray array;
    const ros::Time stamp = ros::Time::now();
    const ros::Duration lifetime(0.2);

    int id = 0;
    for (const auto& obj : obstacles) {
        visualization_msgs::Marker m;
        m.header.frame_id = frame_id;
        m.header.stamp = stamp;
        m.ns = "obstacles";
        m.id = id++;
        m.type = visualization_msgs::Marker::CUBE;
        m.action = visualization_msgs::Marker::ADD;
        m.lifetime = lifetime;

        m.pose.position = MakePoint(obj.x, obj.y, obj.width * 0.3);
        m.pose.orientation = YawToQuaternion(obj.heading);
        m.scale.x = obj.length;
        m.scale.y = obj.width;
        m.scale.z = obj.width * 0.6;
        m.color = MakeColor(0.9f, 0.1f, 0.1f, 0.8f);

        array.markers.push_back(std::move(m));
    }
    return array;
}

void BroadcastEgoTransform(const CartesianState& cs,
                            const std::string& map_frame,
                            const std::string& ego_frame) {
    // static: TransformBroadcaster는 내부적으로 /tf 퍼블리셔를 갖고 있으므로
    // 호출마다 새로 만들지 않고 재사용한다 (ROS 표준 관례).
    static tf2_ros::TransformBroadcaster broadcaster;

    geometry_msgs::TransformStamped t;
    t.header.stamp = ros::Time::now();
    t.header.frame_id = map_frame;
    t.child_frame_id = ego_frame;
    t.transform.translation.x = cs.x;
    t.transform.translation.y = cs.y;
    t.transform.translation.z = 0.0;
    t.transform.rotation = YawToQuaternion(cs.yaw);

    broadcaster.sendTransform(t);
}
