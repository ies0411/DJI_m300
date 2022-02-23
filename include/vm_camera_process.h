#ifndef __CAMERA_H__
#define __CAMERA_H__
#include "vm_common.h"
class VmCameraM300 {
   private:
    /* data */

    ros::NodeHandle nh;
    ros::Subscriber fpv_camera_stream_sub, main_camera_stream_sub, fpv_camera_h264_sub;

    ros::ServiceClient setup_camera_stream_client, setup_camera_h264_client, camera_start_shoot_single_photo_client;
    ros::ServiceServer take_picture_oneshot_server, take_picture_oneshot_sub;
    dji_osdk_ros::SetupCameraStream setupCameraStream_;
    dji_osdk_ros::SetupCameraH264 setupCameraH264_;

    // dji_osdk_ros::CameraStartShootSinglePhoto cameraStartShootSinglePhoto;

    ros::Publisher gimbal_pub;
    void mainCameraStreamCallBack(const sensor_msgs::Image::ConstPtr &msg);
    void fpvCameraStreamCallBack(const sensor_msgs::Image::ConstPtr &msg);
    void cameraH264CallBack(const sensor_msgs::Image::ConstPtr &msg);
    void Vm_Camera_Init(void);
    // void TakePictureOneshot(void);
    //void TakePictureCallback(const std_msgs::Bool::ConstPtr &msg);
    bool TakePictureCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

   protected:
   public:
    VmCameraM300(/* args */);
    ~VmCameraM300();
};

VmCameraM300::VmCameraM300(/* args */) {
    /**service**/
    setup_camera_stream_client = nh.serviceClient<dji_osdk_ros::SetupCameraStream>("setup_camera_stream");
    setup_camera_h264_client = nh.serviceClient<dji_osdk_ros::SetupCameraH264>("setup_camera_h264");
    take_picture_oneshot_sub = nh.advertiseService("take_picture_oneshot", &VmCameraM300::TakePictureCallback, this);
    /**subscribe**/

    //take_picture_oneshot_sub = nh.subscribe("take_picture_oneshot", 10, &VmCameraM300::TakePictureCallback, this);
    fpv_camera_h264_sub = nh.subscribe("dji_osdk_ros/camera_h264_stream", 10, &VmCameraM300::cameraH264CallBack, this);
    fpv_camera_stream_sub = nh.subscribe("dji_osdk_ros/fpv_camera_images", 10, &VmCameraM300::fpvCameraStreamCallBack, this);
    main_camera_stream_sub = nh.subscribe("dji_osdk_ros/main_camera_images", 10, &VmCameraM300::mainCameraStreamCallBack, this);
    /**publish**/
    gimbal_pub = nh.advertise<dji_osdk_ros::Gimbal>("dji_osdk_ros/gimbal_angle_cmd", 10);

    camera_start_shoot_single_photo_client = nh.serviceClient<dji_osdk_ros::CameraStartShootSinglePhoto>("camera_start_shoot_single_photo");

    Vm_Camera_Init();
}

VmCameraM300::~VmCameraM300() {
    ROS_INFO("Basic_command Node close");
}

// #include "vm_basic_command.h"

/**
 * @brief camera raw data & gimbal control process function
 * @details not yet completed
 * @author Ethan.lim
 * @date '21.01.26 
 */

void VmCameraM300::Vm_Camera_Init(void) {
    // setupCameraH264_.request.request_view = setupCameraH264_.request.FPV_CAMERA;
    // setupCameraH264_.request.start = 1;
    // setup_camera_h264_client.call(setupCameraH264_);

    // setupCameraStream_.request.cameraType = setupCameraStream_.request.FPV_CAM;
    // setupCameraStream_.request.start = 1;
    // setup_camera_stream_client.call(setupCameraStream_);

    // setupCameraH264_.request.request_view = setupCameraH264_.request.MAIN_CAMERA;
    // setupCameraH264_.request.start = 1;
    // setup_camera_h264_client.call(setupCameraH264_);
    // if (setupCameraH264_.response.result == false) ROS_ERROR_STREAM("cameraH264 FAILED");

    // setupCameraStream_.request.cameraType = setupCameraStream_.request.MAIN_CAM;
    // setupCameraStream_.request.start = 1;
    // setup_camera_stream_client.call(setupCameraStream_);
}

void VmCameraM300::mainCameraStreamCallBack(const sensor_msgs::Image::ConstPtr &msg) {
    /**todo**/
}

void VmCameraM300::fpvCameraStreamCallBack(const sensor_msgs::Image::ConstPtr &msg) {
    /**todo**/
}

void VmCameraM300::cameraH264CallBack(const sensor_msgs::Image::ConstPtr &msg) {
    /**todo**/
}
/*
void VmCameraM300::TakePictureCallback(const std_msgs::Bool::ConstPtr &msg) {
    //dji_osdk_ros::CameraStartShootAEBPhoto cameraStartShootSinglePhoto;
    dji_osdk_ros::CameraStartShootSinglePhoto cameraStartShootSinglePhoto;
    cameraStartShootSinglePhoto.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
    if ( camera_start_shoot_single_photo_client.call(cameraStartShootSinglePhoto)) {                  
        ROS_INFO_STREAM("sucess");
        camera_flag=true;
        //ROS_INFO_STREAM("AEB photo taken");
    } else {
        ROS_INFO_STREAM("fail");
        camera_flag=false;
    }

    // if (msg->data == true) {
    //     cameraStartShootSinglePhoto.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
    //     if (camera_start_shoot_single_photo_client.call(cameraStartShootSinglePhoto)) {
    //         ROS_INFO_STREAM("take picture sucess");
    //     } else {
    //         ROS_INFO_STREAM("Error take picruey");
    //     }
    // }
}
*/
bool VmCameraM300::TakePictureCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    dji_osdk_ros::CameraStartShootSinglePhoto cameraStartShootSinglePhoto;
    cameraStartShootSinglePhoto.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
    if (camera_start_shoot_single_photo_client.call(cameraStartShootSinglePhoto)) {
        ROS_INFO_STREAM("sucess");
        if (req.data == true) {
            res.success = true;
        } else {
            ROS_INFO_STREAM("Not requested");
        }
        //ROS_INFO_STREAM("AEB photo taken");
    } else {
        ROS_INFO_STREAM("fail");
    }
    return true;
}

#endif