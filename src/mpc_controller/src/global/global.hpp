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
#include <std_msgs/Float32MultiArray.h>

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

    // true면 x_ref[i]/v_ref[i] 등이 "지금부터 i*dt초 후"를 나타내는 시간
    // 인덱스라는 뜻 (planner의 외부 궤적, dt가 MPC dt와 동일). MPC 예측
    // 스텝 i는 위치로 가장 가까운 점을 찾지 말고 ref[i]를 직접 써야 한다 -
    // 정지 상태에서 위치기반 매칭이 항상 index 0(속도 0)에 고정돼 가속
    // 그래디언트가 사라지는 교착상태를 없애기 위함 (cost_function.cpp 참고).
    // CSV(buildReferenceFromWaypoints)는 공간(arc-length) 기반이라 false 유지.
    bool time_indexed = false;

    // ref[0]이 실제로는 "지금"이 아니라 몇 스텝 전 시점을 나타낼 때(예: planner
    // 발행 후 MPC가 쓸 때까지의 처리지연, 또는 EXTERNAL_GRACE로 옛 스냅샷을
    // 재사용하는 동안 계속 흐른 시간) 보정하는 값 - controlLoop에서 매 사이클
    // "실제 경과시간/dt"로 채워 넣는다. ref_idx = i + time_offset_steps로 사용
    // (cost_function.cpp 참고). MORAI 실측으로 확인됨: 이걸 0으로 두면 ref[0]과
    // 실제 ego 위치가 속도에 비례해 벌어지고(고정 시간지연 x 속도), GRACE 중엔
    // 이게 누적돼 결국 조향 발산으로 이어짐 (2026-07-10).
    int time_offset_steps = 0;

    size_t size() const { return x_ref.size(); }
    bool   empty() const { return x_ref.empty(); }
    void   clear() {
        x_ref.clear(); y_ref.clear();
        yaw_ref.clear(); v_ref.clear(); k_ref.clear();
        time_indexed = false;
        time_offset_steps = 0;
    }
};

// ========================================
// planner(frenet_planner_node)가 /frenet_planner/trajectory로 보내는
// 원본 궤적. CartesianPath(planner 쪽)의 [n, x[n], y[n], yaw[n], kappa[n],
// v[n], a[n]] 레이아웃을 그대로 파싱한 결과 (a는 ReferencePath에
// 대응 필드가 없어 MPC가 쓰지 않는다).
// ========================================
struct ExternalTrajectory {
    std::vector<double> x, y, yaw, k, v;

    bool valid() const { return !x.empty(); }
    void clear() { x.clear(); y.clear(); yaw.clear(); k.clear(); v.clear(); }
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

    // accel에도 steering_rate_max와 대칭되는 하드 변화율 제한을 둔다 -
    // 이게 없으면 한 사이클 만에 accel_min<->accel_max로 순간이동이
    // 가능해서, accel_raw가 0 근처에서 아주 조금만 흔들려도(노이즈)
    // 액셀/브레이크 페달이 매 사이클 서로 다른 액추에이터로 뒤바뀌며
    // 깔짝거리는 현상의 근본 원인이 됨 (mpc_node.cpp의 accel/brake
    // 분기 참고). 응급제동 반응성은 유지해야 하므로 전체 range(7.0)를
    // 3~4 사이클 안에는 완주할 수 있는 크기로 잡음.
    double accel_rate_max   = 20.0;    // [m/s^3]

    // accel_raw가 이 범위 안이면 accel/brake 둘 다 0(관성 주행)으로 처리 -
    // 위 rate limit을 넣어도 남을 수 있는 미세한 근접-영점 잡음이 액셀/
    // 브레이크 액추에이터 전환 자체를 유발하지 않도록 하는 마지막 방어선.
    double accel_deadband   = 0.15;    // [m/s²]

    // 비용 가중치
    double weight_path_error    = 5.0;
    double weight_heading_error = 2.0;
    double weight_speed_error   = 0.3;
    double weight_control       = 0.05;
    double weight_control_rate  = 0.5;
    double weight_terminal      = 5.0;   // 마지막 상태 가중치

    // computeControlRateCost의 accel 변화율 가중치 비율(steer 대비). 기존
    // 0.05(=steer의 1/20)는 accel이 사이클마다 크게 바뀌어도 cost가 거의
    // 안 올라 하드리밋(accel_rate_max)에 매번 부딪히며 뚝뚝 끊기게 만드는
    // 원인이었음 - 하드리밋 근처까지 안 가고도 자연스럽게 완만한 해를
    // 찾도록 이 비율을 올림.
    double control_rate_accel_ratio = 0.3;

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

    // planner가 이번 사이클에 유효 경로를 못 내도(n=0) 곧바로 정지하지 않고
    // 이 시간(초) 동안은 직전 유효 외부 경로를 그대로 유지해서 순간적
    // 드롭아웃을 흡수한다. CSV 폴백은 없음(v5 설계) - 이 시간을 넘기면 정지.
    double external_empty_grace_s = 0.4;
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

// planner(frenet_planner_node) 외부 궤적 - v5: 유일한 주행 소스 (CSV 폴백 없음)
extern ExternalTrajectory external_traj;
extern ros::Time          external_traj_stamp;         // external_traj를 실제로 수신한 시각 (age 보정용)
extern std::mutex         external_traj_mutex;
extern ReferencePath      external_ref_last_good;      // grace 동안 유지할 직전 유효 ReferencePath
extern ros::Time          external_ref_last_good_stamp;

#endif // MPC_GLOBAL_HPP
