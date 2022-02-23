// #ifndef __VM_GIMBAL_H__
// #define __VM_GIMBAL_H__


// #include <dji_osdk_ros/Gimbal.h>
// // #include <dji_camera_image.hpp>
// // #include <dji_osdk_ros/SetupCameraStream.h>
// // #include <dji_osdk_ros/SetupCameraH264.h>

// #include <ros/ros.h>
// // #include <sensor_msgs/Image.h>

// #include <stdio.h>
// #include <vector>
// #include <math.h>


// class Vm_Gimbal_Process
// {
// private:
//    /* data */

//    ros::NodeHandle nh;
// //    ros::Subscriber fpv_camera_stream_sub,main_camera_stream_sub,fpv_camera_h264_sub;
   
// //    ros::ServiceClient setup_camera_stream_client,setup_camera_h264_client;
  
// //    dji_osdk_ros::SetupCameraStream setupCameraStream_;
// //    dji_osdk_ros::SetupCameraH264 setupCameraH264_;

//    ros::Publisher gimbal_pub;
// //    void mainCameraStreamCallBack(const sensor_msgs::Image::ConstPtr &msg);
// //    void fpvCameraStreamCallBack(const sensor_msgs::Image::ConstPtr &msg);
// //    void cameraH264CallBack(const sensor_msgs::Image::ConstPtr &msg);
// //    void Vm_Camera_Init(void);
// protected:


// public:
//    Vm_Gimbal_Process(/* args */);
//    ~Vm_Gimbal_Process();  
// };

// Vm_Gimbal_Process::Vm_Gimbal_Process(/* args */)
// {    
    
  
// //    /**service**/
// //    setup_camera_stream_client = nh.serviceClient<dji_osdk_ros::SetupCameraStream>("setup_camera_stream");
// //    setup_camera_h264_client = nh.serviceClient<dji_osdk_ros::SetupCameraH264>("setup_camera_h264");

// //    /**subscribe**/
// //    fpv_camera_h264_sub = nh.subscribe("dji_osdk_ros/camera_h264_stream", 10, &Vm_Camera_Process::cameraH264CallBack,this);
// //    fpv_camera_stream_sub = nh.subscribe("dji_osdk_ros/fpv_camera_images", 10, &Vm_Camera_Process::fpvCameraStreamCallBack,this);
// //    main_camera_stream_sub = nh.subscribe("dji_osdk_ros/main_camera_images", 10, &Vm_Camera_Process::mainCameraStreamCallBack,this);

//    /**publish**/  
//    gimbal_pub = nh.advertise<dji_osdk_ros::Gimbal>("dji_osdk_ros/gimbal_angle_cmd",10);

// //    Vm_Camera_Init();
  

// }

// Vm_Gimbal_Process::~Vm_Gimbal_Process()
// {
//    ROS_INFO("Basic_command Node close");
// }

// #if 0
// Header header
// # ts is the time it takes to achieve the desired angle,
// # so the shorter the time, the faster the gimbal rotates.

// #request
// bool is_reset

// uint8 payload_index

// # rotation cooradiration
// # 0 = execute angle command based on the previously set reference point
// # 1 = execute angle command based on the current point
// uint8 rotationMode

// # pitch angle in degree, unit : deg
// float32 pitch

// # roll angle in degree, unit : deg
// float32 roll

// # yaw angle in degree, unit : deg
// float32 yaw

// #execution time, unit : second
// float64 time

// ---
// #response
// bool result


// void
// DJISDKNode::gimbalAngleCtrlCallback(const dji_osdk_ros::Gimbal::ConstPtr& msg)
// {
//   ROS_DEBUG("called gimbalAngleCtrlCallback");

//   DJI::OSDK::Gimbal::AngleData angle_data;
//   //! OSDK takes 0.1 sec as unit
//   angle_data.duration = msg->ts*10;
//   angle_data.mode     = msg->mode;
//   //! OSDK takes 0.1 deg as unit
//   angle_data.roll     = RAD2DEG(msg->roll)*10;
//   angle_data.pitch    = RAD2DEG(msg->pitch)*10;
//   angle_data.yaw      = RAD2DEG(msg->yaw)*10;
//   vehicle->gimbal->setAngle(&angle_data);
// }

// void
// DJISDKNode::gimbalSpeedCtrlCallback(
//   const geometry_msgs::Vector3Stamped::ConstPtr& msg)
// {
//   ROS_DEBUG("called gimbalAngleCtrlCallback");

//   DJI::OSDK::Gimbal::SpeedData speed_data;
//   //! OSDK takes 0.1 deg as unit
//   speed_data.gimbal_control_authority = 1;
//   speed_data.roll  = RAD2DEG(msg->vector.x)*10;
//   speed_data.pitch = RAD2DEG(msg->vector.y)*10;
//   speed_data.yaw   = RAD2DEG(msg->vector.z)*10;
//   vehicle->gimbal->setSpeed(&speed_data);
// }

// #endif


// #endif