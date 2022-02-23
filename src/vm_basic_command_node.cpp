

#include "vm_basic_command.h"

bool VmDjiM300::CheckParamBusyException(const uint16_t &param_num, const std::vector<std::string> &param, const uint8_t &type) {
    uint8_t check_bit = (ptr_move_m300_->GetStopBusyFlag() & bit_mask);
    if (check_bit & (1 << 1)) {
        ROS_ERROR_STREAM("busy!");
        return false;
    }
    if (type == LinearFlightType::WAYPOINT) {
        if (param.size() % param_num != 2) {
            ROS_ERROR_STREAM("incorrect param number");
            return false;
        }

    } else if (type == LinearFlightType::SIMPLE) {
        if (param.size() != param_num) {
            ROS_ERROR_STREAM("incorrect param number");
            return false;
        }
    }
    ptr_move_m300_->SetStopBusyFlag(busy_bit);
    return true;
}

void VmDjiM300::GetCmdCallback(const std_msgs::String::ConstPtr &msg) {
    std::istringstream iss(msg->data);
    std::string token;
    std::vector<std::string> param;

    while (getline(iss, token, ' ')) {
        param.emplace_back(token);
    }

    if (param[0] == "takeoff") {
        if (!CheckParamBusyException(1, param)) {
            return;
        }
        if (!ptr_move_m300_->TakeOff()) {
            ROS_ERROR_STREAM("takeoff error");
            return;
        }
    }

    else if (param[0] == "landing") {
        if (!CheckParamBusyException(1, param)) {
            return;
        }

        if (!ptr_move_m300_->Landing()) {
            ROS_ERROR_STREAM("landing error");
        }
    }

    else if (param[0] == "landingbymarker") {
        if (!CheckParamBusyException(1, param)) {
            return;
        }
        auto output = std::async(std::launch::async, &VmMove::LandingByMarker, ptr_move_m300_);

        ROS_INFO_STREAM("parents finish");
        //todo//
    } else if (param[0] == "movebypose") {
        if (!CheckParamBusyException(5, param)) {
            return;
        }

        //  todo -> lamda function
        double_t &&tmp_x_goal = std::move(stod(param[1]));
        double_t &&tmp_y_goal = std::move(stod(param[2]));
        double_t &&tmp_z_goal = std::move(stod(param[3]));
        double_t &&tmp_yaw_goal = std::move(stod(param[4]));

        if (POSE_INPUT_EXCEPTION_MACRO(tmp_x_goal, tmp_y_goal, tmp_z_goal)) {
            ROS_ERROR_STREAM("should input until 100 and z be upper than 0");
            return;
        }
        double_t &&tmp_th_goal = stod(param[4]);
        if (tmp_th_goal > 360.0) {
            ROS_ERROR_STREAM("input degree 0~360");
            return;
        }
        ptr_move_m300_->SetGoalPose(std::move(tmp_x_goal), std::move(tmp_y_goal), std::move(tmp_z_goal), std::move(tmp_th_goal));

        auto output = std::async(std::launch::async, &VmMove::MoveByOdom, ptr_move_m300_, LinearFlightType::SIMPLE);

    } else if (param[0] == "waypoint") {
        if (!CheckParamBusyException(4, param, LinearFlightType::WAYPOINT)) {
            return;
        }

        std::shared_ptr<std::vector<Waypoint>> ptr_waypoint_vector = std::make_shared<std::vector<Waypoint>>();

        ptr_waypoint_vector->clear();
        ptr_waypoint_vector->reserve((int)((param.size() - 2 / 4) + 1));
        Waypoint temp;
        for (uint8_t index = 1; index < param.size(); index++) {
            if (param[index] == "finish")
                break;
            if (index % 4 == 1) {
                temp.x_goal = stod(param[index]);
            } else if (index % 4 == 2) {
                temp.y_goal = stod(param[index]);
            } else if (index % 4 == 3) {
                temp.z_goal = stod(param[index]);
            } else if (index % 4 == 0) {
                if (POSE_INPUT_EXCEPTION_MACRO(temp.x_goal, temp.y_goal, temp.z_goal)) {
                    ROS_ERROR_STREAM("should input until 100 and z be upper than 0");
                    return;
                }
                temp.yaw_goal = stod(param[index]);

                if (temp.yaw_goal > 360.0) {
                    ROS_ERROR_STREAM("input degree 0~360");
                    return;
                }
                temp.yaw_goal = DEG_TO_RAD(temp.yaw_goal);
                ptr_waypoint_vector->push_back(temp);
            }
        }
        auto output = std::async(std::launch::async, &VmMove::MoveWaypoint, ptr_move_m300_, std::move(ptr_waypoint_vector));
        ROS_INFO_STREAM("parents finish");
    } else if (param[0] == "RTH") {
        if (!CheckParamBusyException(2, param)) {
            return;
        }

        double &&altitude = stod(param[1]);
        //todo exception
        // if (altitude > 50 || altitude < 3)
        // {
        //     ROS_ERROR_STREAM("Input RTH Altitude from '3' to '50'");
        //     return;
        // }
        auto output = std::async(std::launch::async, &VmMove::ReturnToHome, ptr_move_m300_, std::move(altitude));

    } else if (param[0] == "GPS") {
        if (param.size() != 4) {
            ROS_ERROR_STREAM("incorrect param number");
            return;
        }
        double_t &&latitude = stod(param[1]);
        double_t &&longitude = stod(param[2]);
        double_t &&altitude = stod(param[3]);
        double_t &&tmp_th_goal = stod(param[4]);
        std::shared_ptr<NorthEastCoordinate> NEC_position = ptr_move_m300_->GPSToOdom(latitude, longitude, altitude);
        std::shared_ptr<Eigen::Vector3d> GPS_bias = ptr_move_m300_->GetGPSBias();
        double_t &&tmp_x_goal = NEC_position->easting - (*GPS_bias)[InfoDirType::X];
        double_t &&tmp_y_goal = NEC_position->northing - (*GPS_bias)[InfoDirType::Y];
        double_t &&tmp_z_goal = NEC_position->altitude - (*GPS_bias)[InfoDirType::Z];

        ptr_move_m300_->SetGoalPose(std::move(tmp_x_goal), std::move(tmp_y_goal), std::move(tmp_z_goal), std::move(tmp_th_goal));

        auto output = std::async(std::launch::async, &VmMove::MoveByOdom, ptr_move_m300_, LinearFlightType::SIMPLE);
        ROS_INFO_STREAM("parents finish");
    }

    else if (param[0] == "s") {
        ptr_move_m300_->SetStopBusyFlag(stop_bit);
        ROS_INFO_STREAM("stop bit");
    }

    else {
        std::cout << error_msg << std::endl;
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "VmDjiM300");
    VmDjiM300 vm_basic_cmd;
    ros::Rate rate(50);
    while (ros::ok()) {
        vm_basic_cmd.ptr_move_m300_->SendVelToDrone();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
