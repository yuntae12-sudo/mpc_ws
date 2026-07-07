#include "behavior_planner/node/behavior_node.hpp"

#include <visualization_msgs/MarkerArray.h>

#include "behavior_planner/context/msg_converter.hpp"
#include "behavior_planner/global/utils.hpp"
#include "behavior_planner/node/morai_msg_converter.hpp"

namespace behavior_planner {

BehaviorNode::BehaviorNode(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(std::move(nh)), pnh_(std::move(pnh)) {
  std::string ego_topic = "/Ego_topic";
  std::string objects_topic = "/Object_topic";
  std::string feedback_topic = "/planner/plan_feedback";
  std::string context_topic = "/behavior/context";
  std::string debug_topic = "/behavior/feature_debug";
  std::string marker_topic = "/behavior/markers";
  std::string link_path;
  std::string node_path;
  std::string route_path;
  std::string traffic_light_path;

  pnh_.param("behavior/update_rate", rate_, rate_);
  pnh_.param("topics/ego", ego_topic, ego_topic);
  pnh_.param("topics/objects", objects_topic, objects_topic);
  pnh_.param("topics/plan_feedback", feedback_topic, feedback_topic);
  pnh_.param("publish_topics/behavior_context", context_topic, context_topic);
  pnh_.param("publish_topics/feature_debug", debug_topic, debug_topic);
  pnh_.param("publish_topics/marker", marker_topic, marker_topic);
  pnh_.param("map/link_set_path", link_path, std::string());
  pnh_.param("map/node_set_path", node_path, std::string());
  pnh_.param("route/route_links_path", route_path, std::string());
  pnh_.param("traffic_light/mock_path", traffic_light_path, std::string());

  if (!map_.load(link_path, node_path)) {
    ROS_ERROR("MGeo loading failed; check map paths");
  }
  if (!route_.initialize(map_, loadRouteLinks(route_path))) {
    ROS_ERROR("Route initialization failed; check route_links.yaml");
  }

  auto mock = std::make_shared<MockTrafficLightProvider>();
  if (!mock->load(traffic_light_path)) {
    ROS_WARN("Traffic-light mock loading failed; default GREEN is used");
  }
  lights_ = mock;

  context_pub_ =
      nh_.advertise<behavior_planner::BehaviorContext>(context_topic, 10);
  debug_pub_ = nh_.advertise<UrbanFeatureDebug>(debug_topic, 10);
  marker_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>(marker_topic, 2);
  visualization_ = BehaviorVisualization(marker_pub_);

  ego_sub_ = nh_.subscribe(ego_topic, 5, &BehaviorNode::egoCallback, this);
  objects_sub_ =
      nh_.subscribe(objects_topic, 5, &BehaviorNode::objectsCallback, this);
  feedback_sub_ =
      nh_.subscribe(feedback_topic, 5, &BehaviorNode::feedbackCallback, this);

  ROS_INFO_STREAM("Behavior planner input: ego=" << ego_topic
                  << ", objects=" << objects_topic);
}

void BehaviorNode::spin() {
  ros::Rate loop_rate(rate_);
  while (ros::ok()) {
    ros::spinOnce();
    runOnce();
    loop_rate.sleep();
  }
}

void BehaviorNode::runOnce() {
  if (!ego_received_) {
    ROS_WARN_THROTTLE(5.0, "Waiting for morai_msgs/EgoVehicleStatus");
    return;
  }
  const UrbanFeature feature =
      builder_.build(ego_, objects_, route_, *lights_);
  const BehaviorContextData context = decision_.decide(feature, feedback_);
  context_pub_.publish(MsgConverter::toMsg(context));
  debug_pub_.publish(MsgConverter::toDebugMsg(feature));
  visualization_.publish(feature, context, route_, map_, *lights_);
  logger_.write(feature, context);
}

void BehaviorNode::egoCallback(
    const morai_msgs::EgoVehicleStatus::ConstPtr& message) {
  if (!message) return;
  ego_ = MoraiMsgConverter::toEgoState(*message);
  ego_received_ = true;
}

void BehaviorNode::objectsCallback(
    const morai_msgs::ObjectStatusList::ConstPtr& message) {
  if (!message) return;
  objects_ = MoraiMsgConverter::toPerceptionObjects(*message);
}

void BehaviorNode::feedbackCallback(const PlanFeedback::ConstPtr& message) {
  if (message) feedback_ = *message;
}

}  // namespace behavior_planner
