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
// frenet_planner_node로부터 받는 "PlannedPath" 패킷의 파싱 결과.
// frenet_planner 패키지가 이제 MORAI의 ego/object UDP를 직접 받아서 로컬
// 경로를 계산하고, 그 경로 + 중계용 ego 상태(ego)를 이 프로세스로 보내준다
// (planned_path_receiver.cpp가 파싱). mpc_controller는 더 이상 자체
// EgoInfoReceiver를 두지 않고 이 값을 그대로 쓴다.
// ========================================
struct PlannedPath {
    std::vector<double> x, y, yaw, kappa, v, a;  // frenet_planner의 CartesianPath와 동일 필드
    double dt = 0.0;      // 샘플 간격 [s] (frenet_planner.sample_dt())
    double d = 0.0;       // 진단용: 차선 중앙 기준 횡방향 오프셋
    double d_dot = 0.0;   // 진단용: 횡방향 속도

    // 중계된 ego 상태 (frenet_planner가 MORAI에서 받은 값 그대로 전달)
    double ego_x = 0.0, ego_y = 0.0, ego_yaw = 0.0;
    double ego_v = 0.0, ego_steer = 0.0, ego_accel = 0.0;

    size_t size() const { return x.size(); }
    bool empty() const { return x.empty(); }
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
    // control_rate(steer)와 분리된 accel 변화율 가중치. 예전엔 weight_control_rate*0.3로
    // 파생시켰는데, steer 감쇠만 따로 올려야 하는 상황(지속 곡선 추종 개선)이 생겨서
    // 완전히 분리했다 - yaml에 없으면 이 기본값(옛 15*0.3=4.5와 동일)을 그대로 쓴다.
    double weight_control_rate_accel = 4.5;
    double weight_terminal      = 5.0;   // 마지막 상태 가중치

    // 솔버 hyper-params
    double lr_init      = 0.05;  // 초기 학습률
    double lr_min       = 1e-4;  // 최소 학습률
    double convergence_eps = 1e-3;
    int    line_search_steps = 6;
};

// ========================================
// 전역 변수 (extern)
// ========================================

// 차량 상태
extern MPCState  ego;
extern MPCControl last_control;

// 파라미터
extern MPCParams mpc_params;

#endif // MPC_GLOBAL_HPP
