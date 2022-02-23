
#ifndef __COMMON_H__
#define __COMMON_H__

#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <dji_osdk_ros/CameraAperture.h>
#include <dji_osdk_ros/CameraEV.h>
#include <dji_osdk_ros/CameraFocusPoint.h>
#include <dji_osdk_ros/CameraISO.h>
#include <dji_osdk_ros/CameraRecordVideoAction.h>
#include <dji_osdk_ros/CameraSetZoomPara.h>
#include <dji_osdk_ros/CameraShutterSpeed.h>
#include <dji_osdk_ros/CameraStartShootAEBPhoto.h>
#include <dji_osdk_ros/CameraStartShootBurstPhoto.h>
#include <dji_osdk_ros/CameraStartShootIntervalPhoto.h>
#include <dji_osdk_ros/CameraStartShootSinglePhoto.h>
#include <dji_osdk_ros/CameraStopShootPhoto.h>
#include <dji_osdk_ros/CameraTapZoomPoint.h>
#include <dji_osdk_ros/CameraZoomCtrl.h>
#include <dji_osdk_ros/EmergencyBrake.h>
#include <dji_osdk_ros/FlightTaskControl.h>
#include <dji_osdk_ros/GetAvoidEnable.h>
#include <dji_osdk_ros/Gimbal.h>
#include <dji_osdk_ros/GimbalAction.h>
#include <dji_osdk_ros/ObtainControlAuthority.h>
#include <dji_osdk_ros/SetAvoidEnable.h>
#include <dji_osdk_ros/SetupCameraH264.h>
#include <dji_osdk_ros/SetupCameraStream.h>
#include <dji_osdk_ros/common_type.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <gps_common/conversions.h>
#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/SetBool.h>
#include <stdio.h>
#include <termio.h>
#include <tf/tf.h>
#include <unistd.h>

#include <bitset>
#include <dji_camera_image.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <fstream>
#include <functional>
#include <future>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "vm_pid_control.h"

#define ZERO_INT 0
#define ZERO_DOUBLE 0.0
#define Z_SPEED 1

#define MARKER_CHECK 1
#define ROTATION_CHECK 0
#define ROTATION_POSE_CHECK 0
#define POSE_CHECK 0
#define POSE_THRESHOLD_TYPE_CHECK 0
#define DIS_THRESHOLD_TYPE_CHECK 0

#define ALTITUDE_CHECK 0
#define TAKEOFF_CHECK 1
#define ODOM_CHECK 0
#define COMPLEDTE_ODOM_CHECK 0

#define GOTO_PARAM_CHECK 1
#define PID_CHECK 0
#define BATTERY_STATUS_CHECK 0

#define THREE_POINT_DISTANCE 0.3
// #define TWO_POINT_DISTANCE 0

#define RAD_TO_DEG(RAD) (RAD) * (180.f) / (M_PI)
#define DEG_TO_RAD(DEG) (DEG) * (M_PI) / (180.f)
#define RAD_360 (2 * M_PI)

#define SAT_RANGE_X (1.5f)
#define SAT_RANGE_Y (1.5f)
#define SAT_RANGE_Z (1.5f)
#define SAT_RANGE_YAW DEG_TO_RAD(2)
#define SAT_RANGE_DISTANCE (1.0f)

#define SAT_RANGE_LANDING (1.5f)
#define SAT_LANDING_HEIGHT (0.5f)

#define RTH 1
#define LANDING 2

#define VOLTAGE_THRESHOLD (45800.f)
#define ACC_THRESHOLD (0.0005)

#define INPUT_BOUNDARY (100)
#define POSE_INPUT_EXCEPTION_MACRO(X, Y, Z) ((X > INPUT_BOUNDARY) || (Y > INPUT_BOUNDARY) || (Z > INPUT_BOUNDARY) || (-1 * X > INPUT_BOUNDARY) || (-1 * Y > INPUT_BOUNDARY) || (-1 * Z > INPUT_BOUNDARY) || (Z < 0)) ? (1) : (0)
#define TIMEOUT (100)
#define DEVIA_RANGE(X_N, X_O, DIV) (X_N > (X_O + DIV) || X_N < (X_O - DIV)) ? (X_O = X_N) : 0
#define ACC_ZERO_THRESHOLD(X, Y, Z, K) ((X < Y) ? (Z = 0) : (Z = K))

#define SCAN2_CONST_DIS 3.0
#define SCAN2_Y_VEL 0.3
#define SCAN2_DIS_THRESHOLD 0.30

enum InfoDirType {
    X,
    Y,
    Z,
    YAW,
};
enum InfoRPYType {
    IMU_ROLL,
    IMU_PITCH,
    IMU_YAW,
};

enum BasicMoveType {
    ROTATE,
    LINEAR_MOVE,
    FINAL_ROTATE,
};

enum LinearFlightType {
    SIMPLE,
    WAYPOINT,
    FINAL_POSE,
};

volatile const uint8_t stop_bit = 0x1;
volatile const uint8_t busy_bit = 0x10;

static const uint8_t bit_mask = 0x0011;
struct Waypoint {
    double_t x_goal, y_goal, z_goal, yaw_goal;
};

struct NorthEastCoordinate {
    double_t northing, easting, altitude;
};

static const double_t latitude_for_detection = 20.0;

/**calculation distance between two points**/
void ThreePointDistance(const std::vector<std::pair<double_t, double_t>> &point, double_t &distance) {
    distance = sqrt(pow(point[0].first - point[0].second, 2) + pow(point[1].first - point[1].second, 2));
}

/**calculation between current position and goal position**/
void TwoPointDistance(const std::vector<std::pair<double_t, double_t>> &point, double_t &distance) {
    distance = sqrt(pow(point[0].first - point[0].second, 2) + pow(point[1].first - point[1].second, 2) + pow(point[2].first - point[2].second, 2));
}

void TransRadToPiToPi(double_t &before_rad) {
    while (true) {
        if (before_rad > M_PI) {
            before_rad -= (M_PI * 2.0);

        } else if (before_rad < (-1) * M_PI) {
            before_rad += (M_PI * 2.0);
        } else {
            return;
        }
    }
}

//todo ROS_MSGS

#endif