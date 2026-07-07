#pragma once

#include <ros/ros.h>

#include "behavior_planner/context/behavior_context.hpp"
#include "behavior_planner/feature/urban_feature.hpp"
#include "behavior_planner/map/route_manager.hpp"
#include "behavior_planner/signal/traffic_light_provider.hpp"

namespace behavior_planner {

class BehaviorVisualization {
 public:
  BehaviorVisualization() = default;
  explicit BehaviorVisualization(ros::Publisher publisher)
      : publisher_(std::move(publisher)) {}
  void publish(const UrbanFeature& feature,
               const BehaviorContextData& context,
               const RouteManager& route,
               const MGeoLoader& map,
               const TrafficLightProvider& traffic_lights) const;

 private:
  ros::Publisher publisher_;
};

}  // namespace behavior_planner
