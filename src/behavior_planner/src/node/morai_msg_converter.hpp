#pragma once

#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/ObjectStatus.h>
#include <morai_msgs/ObjectStatusList.h>

#include "behavior_planner/feature/urban_feature.hpp"

namespace behavior_planner {

// This is the only MORAI-specific field mapping layer. If a simulator version
// changes its message definitions, adapt this class without touching feature,
// rule, or decision code.
class MoraiMsgConverter {
 public:
  static EgoState toEgoState(const morai_msgs::EgoVehicleStatus& message);
  static PerceptionObjects toPerceptionObjects(
      const morai_msgs::ObjectStatusList& message);

 private:
  static TrackedObject toTrackedObject(
      const morai_msgs::ObjectStatus& message, ObjectType type);
};

}  // namespace behavior_planner
