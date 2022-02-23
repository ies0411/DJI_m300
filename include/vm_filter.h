
#ifndef __FILTER_H__
#define __FILTER_H__

#include "vm_sensor_callback.h"

class VmFilter : protected VmSensor {
   protected:
    ros::Subscriber imu_sub_, gps_odom_sub_;
    void EKFFilter(const geometry_msgs::Vector3Stamped::ConstPtr &imu, const nav_msgs::Odometry::ConstPtr &gps, const geometry_msgs::Vector3Stamped::ConstPtr &gps_vel);

    // Eigen::Vector3d CurrentVel_;
    // Eigen::Vector4d CurrentPose_;
    Eigen::MatrixXf A_EKF, Q_EKF, R_EKF, H_EKF, X_EKF, Z_EKF, P_EKF, Tr, Xp_EKF, Pp_EKF, K_EKF;
    Eigen::MatrixXf A_UKF, Q_UKF, R_UKF, H_UKF, X_UKF, Z_UKF, P_UKF, Xp_UKF, Pp_UKF, K_UKF, Pz_UKF, Zp_UKF, Xi, W;

    uint8_t X_n, X_m;
    ros::Time filter_begin;

   public:
    VmFilter();
    ~VmFilter();
};
VmFilter::VmFilter(/* args */) {
    CurrentVel_ = Eigen::Vector3d::Zero();
    A_EKF = Eigen::MatrixXf::Zero(6, 6);
    Xp_EKF = Eigen::MatrixXf::Zero(6, 1);
    P_EKF = 3.0 * Eigen::MatrixXf::Identity(6, 6);
    Q_EKF = 0.12 * Eigen::MatrixXf::Identity(6, 6);
    R_EKF = 25.0 * Eigen::MatrixXf::Identity(6, 6);
    H_EKF = Eigen::MatrixXf::Zero(6, 6);
    X_EKF = Eigen::MatrixXf::Zero(6, 1);
    Z_EKF = Eigen::MatrixXf::Zero(6, 1);

    P_UKF = 3.0 * Eigen::MatrixXf::Identity(6, 6);
    Q_UKF = 0.12f * Eigen::MatrixXf::Identity(6, 6);  // Q(6,6)=0.2;
    R_UKF = 25.0f * Eigen::MatrixXf::Identity(6, 6);
    H_UKF = Eigen::MatrixXf::Identity(6, 6);
    X_UKF = Eigen::MatrixXf::Zero(6, 1);
    Z_UKF = Eigen::MatrixXf::Zero(6, 1);

    X_n = X_UKF.rows();
    X_m = Z_UKF.rows();

    Xi = Eigen::MatrixXf::Zero(X_n, 2 * X_n + 1);
    W = Eigen::MatrixXf::Zero(2 * X_n + 1, 1);

    filter_begin = ros::Time::now();

    message_filters::Subscriber<geometry_msgs::Vector3Stamped> acc_sub_(nh_, "dji_osdk_ros/acceleration_ground_fused", 1);
    message_filters::Subscriber<nav_msgs::Odometry> gps_odom_sub_(nh_, "gps_translated_odom", 1);
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> gps_vel_sub_(nh_, "dji_osdk_ros/velocity", 1);
    message_filters::TimeSynchronizer<geometry_msgs::Vector3Stamped, nav_msgs::Odometry, geometry_msgs::Vector3Stamped> sync(acc_sub_, gps_odom_sub_, gps_vel_sub_, 1);
    sync.registerCallback(boost::bind(&VmFilter::EKFFilter, this, _1, _2, _3));
}

VmFilter::~VmFilter() {
    ROS_INFO("filter node close");
}

void VmFilter::EKFFilter(const geometry_msgs::Vector3Stamped::ConstPtr &imu, const nav_msgs::Odometry::ConstPtr &gps, const geometry_msgs::Vector3Stamped::ConstPtr &gps_vel) {
    Eigen::MatrixXf global_gps_vel(2, 1), local_gps_vel(2, 1), Tr(2, 2);
    double &&th = -(IMU_bias_[InfoRPYType::IMU_YAW] - (M_PI / 2.0));
    Tr << cos(th), -sin(th),
        sin(th), cos(th);
    global_gps_vel << gps_vel->vector.x, gps_vel->vector.y;
    local_gps_vel = Tr * global_gps_vel;

    GPS_vel_[InfoDirType::X] = local_gps_vel(0);
    GPS_vel_[InfoDirType::Y] = local_gps_vel(1);
    GPS_vel_[InfoDirType::Z] = gps_vel->vector.z;

    double &&tr_x = gps->pose.pose.position.x - GPS_bias_[InfoDirType::X];
    double &&tr_y = gps->pose.pose.position.y - GPS_bias_[InfoDirType::Y];
    double &&tr_z = gps->pose.pose.position.z - GPS_bias_[InfoDirType::Z];

    Eigen::MatrixXf Temp_pose(2, 1), RT_pose;

    Temp_pose << tr_x, tr_y;
    RT_pose = Tr * Temp_pose;
    GPS_odom_[InfoDirType::X] = RT_pose(0);
    GPS_odom_[InfoDirType::Y] = RT_pose(1);
    GPS_odom_[InfoDirType::Z] = RT_pose(2);

    Eigen::MatrixXf global_acc(2, 1), local_acc(2, 1);
    // th = -(IMU_bias_[InfoRPYType::IMU_YAW] - (M_PI / 2.0));
    // Tr << cos(th), -sin(th),
    //     sin(th), cos(th);
    global_acc << imu->vector.x, imu->vector.y;
    local_acc = Tr * global_acc;
    ACC_odom_[InfoDirType::X] = local_acc(0);
    ACC_odom_[InfoDirType::Y] = local_acc(1);
    ACC_odom_[InfoDirType::Z] = imu->vector.z;

    ros::Time after_filter;
    after_filter = ros::Time::now();
    ros::Duration duration = after_filter - filter_begin;
    double dt = duration.toSec();

    A_EKF << 1.0f, 0.0f, 0.0f, dt, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f, dt, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f, 0.0f, dt,
        0.0f, 0.0f, 0.0f, 1.0, 0.0, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0, 1.0, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f;

    Xp_EKF(0) = CurrentPose_[InfoDirType::X] + dt * CurrentVel_[InfoDirType::X] + 0.5 * ACC_odom_[InfoDirType::X] * dt * dt;
    Xp_EKF(1) = CurrentPose_[InfoDirType::Y] + dt * CurrentVel_[InfoDirType::Y] + 0.5 * ACC_odom_[InfoDirType::Y] * dt * dt;
    Xp_EKF(2) = CurrentPose_[InfoDirType::Z] + dt * CurrentVel_[InfoDirType::Z] + 0.5 * ACC_odom_[InfoDirType::Z] * dt * dt;
    Xp_EKF(3) = CurrentVel_[InfoDirType::X] + ACC_odom_[InfoDirType::X] * dt;
    Xp_EKF(4) = CurrentVel_[InfoDirType::Y] + ACC_odom_[InfoDirType::Y] * dt;
    Xp_EKF(5) = CurrentVel_[InfoDirType::Z] + ACC_odom_[InfoDirType::Z] * dt;
    filter_begin = ros::Time::now();

    Pp_EKF = A_EKF * P_EKF * A_EKF.transpose() + Q_EKF;

    Eigen::MatrixXf temp;

    H_EKF << cos(-IMU_bias_[InfoRPYType::IMU_YAW]), -sin(-IMU_bias_[InfoRPYType::IMU_YAW]), 0, 0, 0, 0,
        sin(-IMU_bias_[InfoRPYType::IMU_YAW]), cos(-IMU_bias_[InfoRPYType::IMU_YAW]), 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, cos(-IMU_bias_[InfoRPYType::IMU_YAW]), -sin(-IMU_bias_[InfoRPYType::IMU_YAW]), 0,
        0, 0, 0, sin(-IMU_bias_[InfoRPYType::IMU_YAW]), cos(-IMU_bias_[InfoRPYType::IMU_YAW]), 0,
        0, 0, 0, 0, 0, 1;

    temp = H_EKF * Pp_EKF * H_EKF.transpose() + R_EKF;
    K_EKF = Pp_EKF * H_EKF.transpose() * temp.inverse();

    Z_EKF(0) = GPS_odom_[InfoDirType::X];
    Z_EKF(1) = GPS_odom_[InfoDirType::Y];
    Z_EKF(2) = GPS_odom_[InfoDirType::Z];
    Z_EKF(3) = GPS_vel_[InfoDirType::X];
    Z_EKF(4) = GPS_vel_[InfoDirType::Y];
    Z_EKF(5) = GPS_vel_[InfoDirType::Z];

    X_EKF = Xp_EKF + K_EKF * (Z_EKF - H_EKF * Xp_EKF);
    CurrentPose_[InfoDirType::X] = X_EKF(0);
    CurrentPose_[InfoDirType::Y] = X_EKF(1);
    CurrentPose_[InfoDirType::Z] = X_EKF(2);
    CurrentVel_[InfoDirType::X] = X_EKF(3);
    CurrentVel_[InfoDirType::Y] = X_EKF(4);
    CurrentVel_[InfoDirType::Z] = X_EKF(5);

    P_EKF = Pp_EKF - K_EKF * H_EKF * Pp_EKF;
}

#endif