#include "behavior_planner/node/morai_msg_converter.hpp"

#include <cmath>
#include <ros/ros.h>

namespace behavior_planner {
namespace {
constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
}

EgoState MoraiMsgConverter::toEgoState(
    const morai_msgs::EgoVehicleStatus& message) {
  EgoState ego;
  // MGeo and EgoVehicleStatus use the same MORAI local map coordinate system.
  ego.x = message.position.x;
  ego.y = message.position.y;
  ego.yaw = message.heading * kDegreesToRadians;
  // MORAI reports longitudinal ego speed in velocity.x in this workspace.
  ego.speed = std::abs(message.velocity.x);
  ego.stamp = message.header.stamp.isZero()
                  ? ros::Time::now().toSec()
                  : message.header.stamp.toSec();
  return ego;
}

TrackedObject MoraiMsgConverter::toTrackedObject(
    const morai_msgs::ObjectStatus& message, ObjectType type) {
  TrackedObject object;
  object.id = message.unique_id;
  object.type = type;
  object.x = message.position.x;
  object.y = message.position.y;
  object.yaw = message.heading * kDegreesToRadians;
  object.speed = std::hypot(message.velocity.x, message.velocity.y);
  object.accel = message.acceleration.x * std::cos(object.yaw) +
                 message.acceleration.y * std::sin(object.yaw);
  // MORAI ObjectStatus.size is treated as (length, width, height).
  object.length = message.size.x > 0.0 ? message.size.x : 4.5;
  object.width = message.size.y > 0.0 ? message.size.y : 1.8;
  return object;
}

PerceptionObjects MoraiMsgConverter::toPerceptionObjects(
    const morai_msgs::ObjectStatusList& message) {
  PerceptionObjects objects;
  objects.vehicles.reserve(message.npc_list.size());
  objects.pedestrians.reserve(message.pedestrian_list.size());
  objects.obstacles.reserve(message.obstacle_list.size());

  for (const auto& object : message.npc_list) {
    objects.vehicles.push_back(toTrackedObject(object, ObjectType::VEHICLE));
  }
  for (const auto& object : message.pedestrian_list) {
    objects.pedestrians.push_back(
        toTrackedObject(object, ObjectType::PEDESTRIAN));
  }
  for (const auto& object : message.obstacle_list) {
    objects.obstacles.push_back(
        toTrackedObject(object, ObjectType::STATIC_OBSTACLE));
  }
  return objects;
}

}  // namespace behavior_planner
