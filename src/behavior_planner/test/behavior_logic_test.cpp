#include <gtest/gtest.h>

#include "behavior_planner/PlanFeedback.h"
#include "behavior_planner/decision/behavior_decision.hpp"
#include "behavior_planner/rule/hard_rule_filter.hpp"

namespace behavior_planner {

TEST(HardRuleFilterTest, GreenLightDoesNotForceStop) {
  UrbanFeature feature;
  feature.signal.has_stop_line = true;
  feature.signal.distance_to_stop_line = 10.0;
  feature.signal.stop_line_s = 50.0;
  feature.signal.light_state = TrafficLightState::GREEN;
  std::string reason;
  const HardConstraint hard = HardRuleFilter().apply(feature, &reason);
  EXPECT_TRUE(hard.stop_line_constraint);
  EXPECT_FALSE(hard.force_stop);
  EXPECT_EQ("no_hard_constraint", reason);
}

TEST(HardRuleFilterTest, RedLightForcesStop) {
  UrbanFeature feature;
  feature.signal.has_stop_line = true;
  feature.signal.distance_to_stop_line = 10.0;
  feature.signal.stop_line_s = 50.0;
  feature.signal.light_state = TrafficLightState::RED;
  std::string reason;
  const HardConstraint hard = HardRuleFilter().apply(feature, &reason);
  EXPECT_TRUE(hard.force_stop);
  EXPECT_DOUBLE_EQ(44.0, hard.stop_before_s);
  EXPECT_NE(std::string::npos, reason.find("traffic_light"));
}

TEST(BehaviorDecisionTest, EmptyRoadSelectsKeep) {
  UrbanFeature feature;
  feature.route.speed_limit = 13.9;
  PlanFeedback feedback;
  const BehaviorContextData context = BehaviorDecision().decide(feature, feedback);
  EXPECT_EQ(BehaviorType::KEEP, context.selected_behavior);
  EXPECT_FALSE(context.enable.follow);
}

TEST(BehaviorDecisionTest, FollowReducesTargetSpeed) {
  UrbanFeature feature;
  feature.ego.speed = 12.0;
  feature.route.speed_limit = 13.9;
  feature.surrounding.has_front_vehicle = true;
  feature.surrounding.front_vehicle.id = 7;
  feature.surrounding.front_vehicle.speed = 4.0;
  feature.surrounding.front_vehicle.gap = 10.0;
  feature.surrounding.front_vehicle.ttc = 2.0;
  feature.surrounding.front_vehicle.relative_speed = 8.0;
  PlanFeedback feedback;
  const BehaviorContextData context = BehaviorDecision().decide(feature, feedback);
  EXPECT_EQ(BehaviorType::FOLLOW, context.selected_behavior);
  EXPECT_LT(context.target.desired_speed, feature.route.speed_limit);
  EXPECT_GE(context.target.desired_speed, 0.0);
}

}  // namespace behavior_planner

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
