#include <ros/ros.h>
#include "behavior_planner/node/behavior_node.hpp"
int main(int argc,char**argv){ros::init(argc,argv,"behavior_planner");behavior_planner::BehaviorNode node(ros::NodeHandle(),ros::NodeHandle("~"));node.spin();return 0;}
