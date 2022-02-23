/**
 * 
 * @author Ethan.lim
 * @brief this package name is vm basic flight
 *        realizing move by pose commanded by user
 *        and exception process 
 *        Next step : sensor fusion IMU & GPS using EKF , Lane traking and obstacle avoid navigation 
 * @version 0.1
 * @date 2021-01-26
 *  * 
 * 
 */

#ifndef __COMMAND_H__
#define __COMMAND_H__

#include "vm_camera_process.h"
#include "vm_move_function.h"

const std::string error_msg = R"(
   Command is not correct!!!!
)";

class VmDjiM300 {
   private:
    /* data */

    ros::NodeHandle nh;
    ros::Subscriber keyboard_sub;

    bool bat_flag = false, gps_flag = false, flight_flag = false;
    bool stop_flag = false;
    bool current_move_check_flag = false;
    uint8_t emergency_flag = RTH;
    bool Marker_Landing_Working_ = false;

    bool CheckParamBusyException(const uint16_t &param_num, const std::vector<std::string> &param, const uint8_t &type = LinearFlightType::SIMPLE);

    void GetCmdCallback(const std_msgs::String::ConstPtr &msg);

   public:
    VmDjiM300(/* args */);
    ~VmDjiM300();
    std::shared_ptr<VmMove> ptr_move_m300_ = std::make_shared<VmMove>();
};

VmDjiM300::VmDjiM300(/* args */) {
    keyboard_sub = nh.subscribe("keyboard_command", 1, &VmDjiM300::GetCmdCallback, this);
}

VmDjiM300::~VmDjiM300() {
    ROS_INFO("Basic_command Node close");
}

#endif
//    std_srvs::SetBool takepicture_bool;
//    dji_osdk_ros::ObtainControlAuthority obtainCtrlAuthority;
// dji_osdk_ros::SetAvoidEnable upward_avoid_req;
// dji_osdk_ros::GetAvoidEnable getAvoidEnable;
/***publish**/
// cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("flight_ctrl_vel", 10);
// //take_picture_oneshot_pub = nh.advertise<std_msgs::Bool>("take_picture_oneshot", 10);

// /**subscribe**/
// aruco_marker_pose_sub = nh.subscribe("vm_aruco_marker_pose", 10, &VmDjiM300::ArucoMarkerPose, this);
// battery_state_sub = nh.subscribe("dji_osdk_ros/battery_state", 10, &VmDjiM300::Battery_State_Callback, this);
// gps_health_sub = nh.subscribe("dji_osdk_ros/gps_health", 10, &VmDjiM300::GPS_State_Callback, this);
// flight_status_sub = nh.subscribe("dji_osdk_ros/flight_status", 10, &VmDjiM300::Flight_State_Callback, this);
// gps_odom_sub = nh.subscribe("vm_odom", 1, &VmDjiM300::Gps_To_Odom_Callback, this);
// imu_odom_sub = nh.subscribe("dji_osdk_ros/imu", 10, &VmDjiM300::Imu_To_Odom_Callback, this);
// ar_marker_sub = nh.subscribe("ar_pose_marker", 10, &VmDjiM300::Ar_Marker_Callback, this);
// local_pose_sub = nh.subscribe("dji_osdk_ros/local_position", 10, &VmDjiM300::Local_Pose_Callback, this);
// lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan_laser", 1000, &VmDjiM300::LidarScanCallback, this);
// lidar_sub_3D = nh.subscribe<sensor_msgs::PointCloud>("/scan_pcl", 1000, &VmDjiM300::LidarScanCallback_3D, this);
// /**service**/
// take_picture_oneshot_pub = nh.serviceClient<std_srvs::SetBool>("take_picture_oneshot");
// task_control_client = nh.serviceClient<dji_osdk_ros::FlightTaskControl>("flight_task_control");
// get_avoid_enable_client = nh.serviceClient<dji_osdk_ros::GetAvoidEnable>("get_avoid_enable_status");
// enable_upward_avoid_client = nh.serviceClient<dji_osdk_ros::SetAvoidEnable>("/set_upwards_avoid_enable");
// obtain_ctrl_authority_client = nh.serviceClient<dji_osdk_ros::ObtainControlAuthority>("obtain_release_control_authority");

/**init**/
// InitAdvertiseSubscribe();

// ros::ServiceClient set_go_home_altitude_client, enable_upward_avoid_client,
//     get_avoid_enable_client, emergency_brake_client, obtain_ctrl_authority_client, take_picture_oneshot_pub;

// ros::Subscriber lidar_sub, lidar_sub_3D;