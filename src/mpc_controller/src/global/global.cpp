#include "global.hpp"

// ========================================
// 전역 변수 정의 (단일 소스)
// ========================================

// 차량 상태
MPCState   ego;
MPCControl last_control;

// 경로/맵 데이터
std::vector<Waypoint> waypoints;

// 파라미터
MPCParams mpc_params;

// /Ego_topic 최초 수신 여부
bool ego_received = false;

// 진단/플래그
int  closest_waypoint_idx = 0;

// ROS
ros::Publisher cmd_pub;
std::mutex ego_mutex;

// CSV 경로 기본값 (PlanningControl 과 동일)
std::string g_waypoint_file_path = "src/main/config/track_log_recorded_final.csv";
