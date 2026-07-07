#pragma once

#include <memory>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/ObjectStatusList.h>
#include <ros/ros.h>

#include "behavior_planner/PlanFeedback.h"
#include "behavior_planner/decision/behavior_decision.hpp"
#include "behavior_planner/feature/feature_builder.hpp"
#include "behavior_planner/global/data_logger.hpp"
#include "behavior_planner/map/mgeo_loader.hpp"
#include "behavior_planner/signal/mock_traffic_light_provider.hpp"
#include "behavior_planner/visualization/behavior_visualization.hpp"

namespace behavior_planner {

class BehaviorNode {
 public:
  BehaviorNode(ros::NodeHandle, ros::NodeHandle);
  void spin();

 private:
  void runOnce();
  void egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& message);
  void objectsCallback(const morai_msgs::ObjectStatusList::ConstPtr& message);
  void feedbackCallback(const PlanFeedback::ConstPtr& message);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber ego_sub_;
  ros::Subscriber objects_sub_;
  ros::Subscriber feedback_sub_;
  ros::Publisher context_pub_;
  ros::Publisher debug_pub_;
  ros::Publisher marker_pub_;
  MGeoLoader map_;
  RouteManager route_;
  FeatureBuilder builder_;
  BehaviorDecision decision_;
  std::shared_ptr<TrafficLightProvider> lights_;
  BehaviorVisualization visualization_;
  DataLogger logger_;
  EgoState ego_;
  PerceptionObjects objects_;
  PlanFeedback feedback_;
  bool ego_received_ = false;
  double rate_ = 10.0;
};

}  // namespace behavior_planner
