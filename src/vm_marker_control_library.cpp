
#include "vm_basic_command.h"
// #include <vm_basic_cmd_library.h>

/**
 * @brief Landing by using a ar_tracking marker
 * @details Judge current position depending on marker position
 * 
 * @author Ethan.lim
 * @date '21.01.26 
 * 
 */
bool Vm_Basic_Command::Landingbymarker(void) {
    //  if(Marker_Detect_flag){
    //   ros::Rate landing_rate(5);
    //    linear_speed_pid.PID_set(pid_dt,pid_max,pid_min,pid_Kp,pid_Kd,pid_Ki);
    //    th_w_pid.PID_set(pid_th_dt,pid_th_max,pid_th_min,pid_th_Kp,pid_th_Kd,pid_th_Ki);
    //    z_speed_pid.PID_set(pid_dt,pid_max,pid_min,pid_Kp,pid_Kd,pid_Ki);

    //    info_pose[_INFO_X_GOAL]=ZERO_DOUBLE;
    //    info_pose[_INFO_Y_GOAL]=ZERO_DOUBLE;
    //    info_pose[_INFO_Z_GOAL]=SAT_RANGE_LANDING;
    //    info_pose[_INFO_TH_GOAL]=ZERO_DOUBLE;
    //    info_pose[_INFO_SAT_RANGE_W]=SAT_RANGE_YAW;
    //    info_pose[_INFO_SAT_RANGE_X]=SAT_RANGE_X;
    //    info_pose[_INFO_SAT_RANGE_Y]=SAT_RANGE_Y;
    //    info_pose[_INFO_SAT_RANGE_Z]=SAT_RANGE_Z;

    //   while(ros::ok()){
    //    ROS_INFO("landing");
    //    info_pose[_INFO_X]=marker_x;
    //    info_pose[_INFO_Y]=marker_y;
    //    info_pose[_INFO_Z]=marker_z;
    //    info_pose[_INFO_TH_RAD]=marker_th;
    //    info_pose[_INFO_TH_DEG]=marker_th_deg;
    //    info_pose[_INFO_SPEED]=speed;
    //    info_pose[_INFO_TURN]=turn;

    //    if(Linear_Flight_Algorithm(info_pose)){
    //       twist.linear.x = ZERO_DOUBLE;
    //       twist.linear.y = ZERO_DOUBLE;
    //       twist.linear.z = ZERO_DOUBLE;
    //       twist.angular.z = ZERO_DOUBLE;
    //       cmd_vel_pub.publish(twist);
    //       control_task.request.task = dji_osdk_ros::FlightTaskControl::Request::TASK_LAND;
    //       task_control_client.call(control_task);
    //       if(control_task.response.result == true){
    //          ROS_INFO_STREAM("Land task successful");
    //          return true;
    //       }
    //       ROS_INFO_STREAM("Land task failed.");
    //       return false;

    //    }

    //    x_vel=info_pose[_INFO_X_VEL];
    //    y_vel=info_pose[_INFO_Y_VEL];
    //    z_vel=info_pose[_INFO_Z_VEL];
    //    th_vel=info_pose[_INFO_TH_VEL];
    //    marker_speed = info_pose[_INFO_SPEED];
    //    twist.linear.x = x_vel;
    //    twist.linear.y = y_vel;
    //    twist.linear.z = z_vel;
    //    twist.angular.z = th_vel;

    //    cmd_vel_pub.publish(twist);
    //    ros::spinOnce();
    //    landing_rate.sleep();
    //    }
    // }
    // ROS_INFO_STREAM("can't detect marker!");
    // return false;
}

void Vm_Basic_Command::ArucoMarkerPose(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    aruco_x = msg->pose.position.x;
    aruco_y = msg->pose.position.y;
    aruco_z = msg->pose.position.z;

    aruco_time_begin = ros::Time::now();
    // Aruco_Flag = true;
    // ros::Time time_begin = ros::Time::now();
    // while(ros::ok()){
    // ROS_INFO_STREAM("marker_check");

    //ros::Duration duration = time_end - time_begin;
}

void Vm_Basic_Command::Ar_Marker_Callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg) {
    // Marker_Detect_flag = true;
    // double roll, pitch, yaw;
    // for (auto &p : msg->markers) {
    //     marker_x = -p.pose.pose.position.z;
    //     marker_y = -p.pose.pose.position.y;
    //     marker_z = -p.pose.pose.position.x;

    //     tf::Quaternion q(
    //         p.pose.pose.orientation.x,
    //         p.pose.pose.orientation.y,
    //         p.pose.pose.orientation.z,
    //         p.pose.pose.orientation.w);
    //     tf::Matrix3x3 m(q);
    //     m.getRPY(roll, pitch, yaw);
    //     if (pitch < 0) pitch = 2.0 * M_PI + pitch;
    //     marker_th = pitch;
    //     marker_th_deg = RAD_TO_DEG(pitch);
#if MARKER_CHECK
    ROS_INFO("****MARKER POSE*********");
    // ROS_INFO("roll:%f , pitch :%f ,yaw :%f ",roll,pitch,yaw);
    ROS_INFO("x:%f, y:%f, z:%f, yaw:%lf ", marker_x, marker_y, marker_z, RAD_TO_DEG(pitch));
//  ROS_INFO("x:%f, y:%f, z:%f, yas:%f",x,y,z,th_deg);
#endif
}
}
