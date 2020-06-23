//
// Created by amy on 6/23/20.
//

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <joint_calibration/CalibrationData.h>
#include <joint_calibration/Utils.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv,"joint_calibration");
    ros::NodeHandle nh("~");

    // The calibration data
    std_msgs::String description_msg;
    std::vector<joint_calibration::CalibrationData> data;

    // Load calibration data from bagfile
    std::string data_bag_name("/tmp/calibration_data.bag");
    if (argc > 2)
        data_bag_name = argv[2];
    ROS_INFO_STREAM("Loading calibration data from " << data_bag_name);

    if (!joint_calibration::loadBag(data_bag_name, description_msg, data)) {
        // Error will have been printed in function
        return -1;
    }
}