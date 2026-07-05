#include "global/global.hpp"
#include "global/parameter_loader.hpp"
#include "node/mpc_node.hpp"

// ========================================
// main
// ========================================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "mpc_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ROS_INFO("========================================");
    ROS_INFO("    MPC Node Starting");
    ROS_INFO("========================================");

    // 파라미터 로드
    loadMPCParameters(pnh);

    // waypoints 로드 (실패 시 종료 안 함 - 안전상 정지 모드로 실행)
    if (!loadWaypoints()) {
        ROS_ERROR("[MPC] Waypoint load failed. Node will keep running but will publish stop commands.");
    }

    // 초기값
    ego.x = 0.0; ego.y = 0.0; ego.yaw = 0.0; ego.vx = 0.0;
    last_control.delta = 0.0; last_control.accel = 0.0;

    // 구독자
    ros::Subscriber gps_sub      = nh.subscribe("/gps",         1, CBGps);
    ros::Subscriber imu_sub      = nh.subscribe("/imu",         1, CBImu);
    ros::Subscriber ego_sub      = nh.subscribe("/Ego_topic",   1, CBEgoState);

    ROS_INFO("[MPC] Subscribed: /gps /imu /Ego_topic");

    // 발행자
    cmd_pub = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 1);

    // 제어 타이머
    double freq = mpc_params.control_frequency;
    if (freq <= 0.0) freq = 10.0;
    ros::Timer timer = nh.createTimer(ros::Duration(1.0 / freq), controlLoop);

    ROS_INFO("[MPC] Control loop @ %.1f Hz", freq);
    ROS_INFO("========================================");

    // PlanningControl 과 동일하게 멀티스레드 spinner
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
