
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>

#include <sstream>
#include <string>
#include <vector>
/**
 * @brief Publish command line
 * 
 * @details this node is used to pusblish command line by geting vaules from keyboard
 * @author Ethan.lim
 * @date '21.01.26 
 *  
 */

int main(int argc, char *argv[]) {
    //init ROS node

    ros::init(argc, argv, "vm_command_keyboard");
    ros::NodeHandle nh;

    ros::Publisher keyboard_pub = nh.advertise<std_msgs::String>("keyboard_command", 1);

    while (ros::ok()) {
        /* code for loop body */
        std_msgs::String msg;
        getline(std::cin, msg.data);
        if (msg.data[0] == '\x03')
            ros::spinOnce();
        keyboard_pub.publish(msg);
    }
    return 0;
}


