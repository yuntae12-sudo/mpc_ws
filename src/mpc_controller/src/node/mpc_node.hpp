#ifndef MPC_NODE_HPP
#define MPC_NODE_HPP

#include "../global/global.hpp"
#include "../solver/mpc_solver.hpp"
#include "../planner/path_planner.hpp"   // Planner 분리

// ========================================
// ROS 콜백
// ========================================
void CBEgoState(const morai_msgs::EgoVehicleStatus::ConstPtr& msg);

// ========================================
// 제어 루프 (Controller 역할만)
// ========================================
void controlLoop(const ros::TimerEvent&);

// ========================================
// CtrlCmd 발행
// ========================================
void publishCtrlCmd(double steering_rad,
                    double accel_norm,
                    double brake_norm);

#endif // MPC_NODE_HPP