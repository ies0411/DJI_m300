
#ifndef __SENSOR_CALLBACK_H__
#define __SENSOR_CALLBACK_H__

#include "vm_common.h"

class VmSensor {
   protected:
    ros::Time acc_time_;
    ros::NodeHandle nh_;
    uint8_t IMU_cnt_ = 0, GPS_cnt_ = 0;

    ros::Subscriber velocity_sub_, acceleration_sub_, imu_sub_, gps_odom_sub_;

    void IMUCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void GPSCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void GPSVelCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
    void AccelerationCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);

    std::shared_ptr<std::vector<double_t>> ptr_roll_vec_ = std::make_shared<std::vector<double_t>>();
    std::shared_ptr<std::vector<double_t>> ptr_pitch_vec_ = std::make_shared<std::vector<double_t>>();
    std::shared_ptr<std::vector<double_t>> ptr_yaw_vec_ = std::make_shared<std::vector<double_t>>();

    std::shared_ptr<std::vector<double_t>> ptr_gps_x_vec_ = std::make_shared<std::vector<double_t>>();
    std::shared_ptr<std::vector<double_t>> ptr_gps_y_vec_ = std::make_shared<std::vector<double_t>>();
    std::shared_ptr<std::vector<double_t>> ptr_gps_z_vec_ = std::make_shared<std::vector<double_t>>();

    Eigen::Vector4d IMU_odom_, IMU_bias_, ACC_odom_;
    Eigen::Vector3d GPS_odom_, GPS_bias_, GPS_vel_, CurrentVel_;
    Eigen::Vector4d CurrentPose_;

   public:
    VmSensor(/* args */);
    ~VmSensor();

    friend void TransRadToPiToPi(double_t &before_rad);
};

VmSensor::VmSensor() {
    ptr_roll_vec_->reserve(51);
    ptr_pitch_vec_->reserve(51);
    ptr_yaw_vec_->reserve(51);

    ptr_gps_x_vec_->reserve(51);
    ptr_gps_y_vec_->reserve(51);
    ptr_gps_z_vec_->reserve(51);

    IMU_odom_ = Eigen::Vector4d::Zero();
    IMU_bias_ = Eigen::Vector4d::Zero();

    GPS_odom_ = Eigen::Vector3d::Zero();
    GPS_bias_ = Eigen::Vector3d::Zero();

    acceleration_sub_ = nh_.subscribe("dji_osdk_ros/acceleration_ground_fused", 10, &VmSensor::AccelerationCallback, this);
    imu_sub_ = nh_.subscribe("dji_osdk_ros/imu", 10, &VmSensor::IMUCallback, this);
    gps_odom_sub_ = nh_.subscribe("gps_translated_odom", 10, &VmSensor::GPSCallback, this);

    acc_time_ = ros::Time::now();
}

VmSensor::~VmSensor() {
    ROS_INFO("sensor node close");
}
/**
 * @brief acquire roll,pitch,yaw data of body
 * 
 * @param msg --> imu topic msg .. Roll,pitch,yaw
 */

void VmSensor::IMUCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw, temp;
    m.getRPY(roll, pitch, yaw);

    if (IMU_cnt_ > 50) {
        IMU_odom_[InfoRPYType::IMU_YAW] = yaw - IMU_bias_[InfoRPYType::IMU_YAW];
        IMU_odom_[InfoRPYType::IMU_ROLL] = roll - IMU_bias_[InfoRPYType::IMU_ROLL];
        IMU_odom_[InfoRPYType::IMU_PITCH] = pitch - IMU_bias_[InfoRPYType::IMU_PITCH];

    } else if (IMU_cnt_ == 50) {
        std::sort(ptr_roll_vec_->begin(), ptr_roll_vec_->end());
        std::sort(ptr_pitch_vec_->begin(), ptr_pitch_vec_->end());
        std::sort(ptr_yaw_vec_->begin(), ptr_yaw_vec_->end());
        for (int i = 10; i < 40; i++) {
            IMU_bias_[InfoRPYType::IMU_YAW] += ptr_yaw_vec_->at(i);
            IMU_bias_[InfoRPYType::IMU_ROLL] += ptr_roll_vec_->at(i);
            IMU_bias_[InfoRPYType::IMU_PITCH] += ptr_pitch_vec_->at(i);
        }
        IMU_bias_[InfoRPYType::IMU_YAW] /= 30;
        IMU_bias_[InfoRPYType::IMU_ROLL] /= 30;
        IMU_bias_[InfoRPYType::IMU_PITCH] /= 30;
        IMU_cnt_++;
    } else {
        ptr_roll_vec_->push_back(roll);
        ptr_pitch_vec_->push_back(pitch);
        ptr_yaw_vec_->push_back(yaw);
        IMU_cnt_++;
    }
}

void VmSensor::GPSCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    if (GPS_cnt_ > 50) {
        return;
    } else if (GPS_cnt_ == 50) {
        std::sort(ptr_gps_x_vec_->begin(), ptr_gps_x_vec_->end());
        std::sort(ptr_gps_y_vec_->begin(), ptr_gps_y_vec_->end());
        std::sort(ptr_gps_z_vec_->begin(), ptr_gps_z_vec_->end());

        for (int i = 10; i < 40; i++) {
            GPS_bias_[InfoDirType::X] += ptr_gps_x_vec_->at(i);
            GPS_bias_[InfoDirType::Y] += ptr_gps_y_vec_->at(i);
            GPS_bias_[InfoDirType::Z] += ptr_gps_z_vec_->at(i);
        }
        GPS_bias_[InfoDirType::X] /= 30;
        GPS_bias_[InfoDirType::Y] /= 30;
        GPS_bias_[InfoDirType::Z] /= 30;
        ++GPS_cnt_;
    } else {
        ptr_gps_x_vec_->push_back(msg->pose.pose.position.x);
        ptr_gps_y_vec_->push_back(msg->pose.pose.position.y);
        ptr_gps_z_vec_->push_back(msg->pose.pose.position.z);
        ++GPS_cnt_;
    }
}

void VmSensor::AccelerationCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
    Eigen::MatrixXf global_acc(2, 1), local_acc(2, 1), Tr(2, 2);
    double th = -(IMU_bias_[InfoRPYType::IMU_YAW] - (M_PI / 2.0));
    Tr << cos(th), -sin(th),
        sin(th), cos(th);
    global_acc << msg->vector.x, msg->vector.y;
    local_acc = Tr * global_acc;
    ACC_odom_[InfoDirType::X] = local_acc(0);
    ACC_odom_[InfoDirType::Y] = local_acc(1);
    ACC_odom_[InfoDirType::Z] = msg->vector.z;
    ros::Duration duration = ros::Time::now() - acc_time_;
    double_t dt = duration.toSec();

    CurrentPose_[InfoDirType::X] = CurrentPose_[InfoDirType::X] + CurrentVel_[InfoDirType::X] * dt + 0.5 * ACC_odom_[InfoDirType::X] * dt * dt;
    CurrentPose_[InfoDirType::Y] = CurrentPose_[InfoDirType::Y] + CurrentVel_[InfoDirType::Y] * dt + 0.5 * ACC_odom_[InfoDirType::Y] * dt * dt;
    CurrentPose_[InfoDirType::Z] = CurrentPose_[InfoDirType::Z] + CurrentVel_[InfoDirType::Z] * dt + 0.5 * ACC_odom_[InfoDirType::Z] * dt * dt;
    CurrentVel_[InfoDirType::X] = CurrentVel_[InfoDirType::X] + ACC_odom_[InfoDirType::X] * dt;
    CurrentVel_[InfoDirType::Y] = CurrentVel_[InfoDirType::Y] + ACC_odom_[InfoDirType::Y] * dt;
    CurrentVel_[InfoDirType::Z] = CurrentVel_[InfoDirType::Z] + ACC_odom_[InfoDirType::Z] * dt;

    acc_time_ = ros::Time::now();
}

// void VmSensor::Local_Pose_Callback(const geometry_msgs::PointStamped::ConstPtr &msg) {
//     /**todo**/
// }

#endif