#include "global.hpp"

// ========================================
// 전역 변수 정의 (단일 소스)
// ========================================

// 차량 상태
MPCState   ego;
MPCControl last_control;

// 경로/맵 데이터
std::vector<Waypoint> waypoints;
ReferencePath reference_path;
CostmapInfo   costmap_info;

// 파라미터
MPCParams mpc_params;

// GPS 좌표 reference
CoordinateReference coord_ref;
bool coord_ref_initialized = false;
bool gps_jamming_perception = false;

// 진단/플래그
bool new_costmap_received = false;
bool new_reference_path_received = false;
int  closest_waypoint_idx = 0;

// ROS
ros::Publisher cmd_pub;
std::mutex costmap_mutex;
std::mutex ego_mutex;

// CSV 경로 기본값 (PlanningControl 과 동일)
std::string g_ref_file_path      = "src/main/config/ref.txt";
std::string g_waypoint_file_path = "src/main/config/track_log_recorded_final.csv";
