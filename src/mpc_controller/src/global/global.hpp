#ifndef MPC_GLOBAL_HPP
#define MPC_GLOBAL_HPP

#include <vector>
#include <string>
#include <cmath>
#include <mutex>
#include <algorithm>
#include <limits>
#include <fstream>
#include <sstream>
#include <iostream>
#include <array>

#include <ros/ros.h>
#include <morai_msgs/CtrlCmd.h>
#include <morai_msgs/EgoVehicleStatus.h>

// ========================================
// 기본 구조체
// ========================================

// CSV에서 로드되는 reference waypoint
struct Waypoint {
    double x = 0.0;
    double y = 0.0;
    double curvature = 0.0;
};

// ========================================
// 차량 상태 (MPC State)
// ========================================
struct MPCState {
    double x = 0.0;        // X 좌표 [m] (global ENU)
    double y = 0.0;        // Y 좌표 [m] (global ENU)
    double yaw = 0.0;      // Yaw 각도 [rad]
    double vx = 0.0;       // 종방향 속도 [m/s]
};

// ========================================
// 제어 입력
// ========================================
struct MPCControl {
    double delta = 0.0;    // 조향각 [rad]
    double accel = 0.0;    // 가속도 [m/s²]
};

// MPC 결과
struct MPCResult {
    MPCControl control;
    std::vector<MPCState>   predicted_states;
    std::vector<MPCControl> controls;
    double cost = 0.0;
    bool   success = false;
    std::string solver_msg;
};

// ========================================
// 참조 경로 (현재 ego 위치 기준 window)
// ========================================
struct ReferencePath {
    std::vector<double> x_ref;     // global X [m]
    std::vector<double> y_ref;     // global Y [m]
    std::vector<double> yaw_ref;   // global yaw [rad]
    std::vector<double> v_ref;     // 목표 속도 [m/s]
    std::vector<double> k_ref;     // 곡률 [1/m]

    size_t size() const { return x_ref.size(); }
    bool   empty() const { return x_ref.empty(); }
    void   clear() {
        x_ref.clear(); y_ref.clear();
        yaw_ref.clear(); v_ref.clear(); k_ref.clear();
    }
};

// ========================================
// MPC 파라미터
// ========================================
struct MPCParams {
    // 예측 파라미터
    int    horizon         = 15;     // 예측 horizon (steps)
    double dt              = 0.1;    // 샘플링 간격 [s]
    int    max_iterations  = 25;     // 솔버 최대 반복
    double wheelbase       = 3.0;    // 축거 [m]
    double control_frequency = 20.0; // 제어 주기 [Hz]

    // 제약 조건
    double steering_max     = 0.6109;  // 35deg [rad]
    double steering_rate_max= 1.0;     // [rad/s]
    double accel_min        = -5.0;    // [m/s²]
    double accel_max        = 3.0;     // [m/s²]
    double vel_min          = 0.0;     // [m/s]
    double vel_max          = 30.0;    // [m/s]

    // 비용 가중치
    double weight_path_error    = 5.0;
    double weight_heading_error = 2.0;
    double weight_speed_error   = 0.3;
    double weight_control       = 0.05;
    double weight_control_rate  = 0.5;
    double weight_terminal      = 5.0;   // 마지막 상태 가중치

    // 솔버 hyper-params
    double lr_init      = 0.05;  // 초기 학습률
    double lr_min       = 1e-4;  // 최소 학습률
    double convergence_eps = 1e-3;
    int    line_search_steps = 6;

    // 경로/속도 plan
    int    ref_window      = 60;   // 현재 위치 앞쪽으로 사용할 waypoint 수
    double target_vel      = 60.0 / 3.6;  // 기본 목표 속도 [m/s]
    double curve_vel_sharp = 30.0 / 3.6;
    double curve_vel_mid   = 35.0 / 3.6;
    double curve_vel_mild  = 45.0 / 3.6;
    double curve_th_sharp  = 0.01;
    double curve_th_mid    = 0.004;
    double curve_th_mild   = 0.001;
    double curve_lookahead_m = 15.0;  // [m] 이 거리 안에 더 급한 커브가 있으면 미리 그 커브 속도로 감속
};

// ========================================
// 전역 변수 (extern)
// ========================================

// 차량 상태
extern MPCState  ego;
extern MPCControl last_control;

// 경로/맵 데이터
extern std::vector<Waypoint> waypoints;

// 파라미터
extern MPCParams mpc_params;

// /Ego_topic 최초 수신 여부 (수신 전엔 정지 명령만 발행)
extern bool ego_received;

// 진단/플래그
extern int  closest_waypoint_idx;

// ROS
extern ros::Publisher cmd_pub;
extern std::mutex ego_mutex;

// CSV 경로
extern std::string g_waypoint_file_path;

#endif // MPC_GLOBAL_HPP
