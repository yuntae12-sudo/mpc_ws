#ifndef MPC_NODE_HPP
#define MPC_NODE_HPP

#include "../global/global.hpp"
#include "../solver/mpc_solver.hpp"

// ========================================
// ROS 콜백
// ========================================
void CBGps     (const morai_msgs::GPSMessage::ConstPtr& msg);
void CBImu     (const sensor_msgs::Imu::ConstPtr& msg);
void CBEgoState(const morai_msgs::EgoVehicleStatus::ConstPtr& msg);
void CBCostmap (const nav_msgs::OccupancyGrid::ConstPtr& msg);
void CBLanePath(const std_msgs::Float32MultiArray::ConstPtr& msg);  // 선택적 사용

// ========================================
// 제어 루프
// ========================================
void controlLoop(const ros::TimerEvent&);

// ========================================
// Reference path 빌더 (현재 ego 주변 window)
// ========================================
void buildReferenceFromWaypoints();

// ========================================
// 종방향 PID  (m/s 오차 -> normalized accel/brake [0,1])
// ========================================
void velocityPID(double v_target, double v_current,
                 double& out_accel_norm, double& out_brake_norm);

// ========================================
// CtrlCmd 발행
// ========================================
void publishCtrlCmd(double steering_rad,
                    double accel_norm,
                    double brake_norm);

#endif // MPC_NODE_HPP
