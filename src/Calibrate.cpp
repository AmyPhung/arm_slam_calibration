//
// Created by amy on 6/23/20.
//

#include <ros/ros.h>
// ROS Messages
#include <std_msgs/String.h>
#include <joint_calibration/CalibrationData.h>
// Custom Libraries
#include <joint_calibration/ParameterManager.h>
#include <joint_calibration/Utils.h>
#include <joint_calibration/Optimizer.h>
#include <joint_calibration/ChainModel.h>



int main(int argc, char** argv)
{
    // ROS setup
    ros::init(argc, argv,"joint_calibration");
    ros::NodeHandle nh("~");


    // The calibration data
    std_msgs::String description_msg;
    joint_calibration::CalibrationData data;

    // Load calibration data from bagfile
    std::string data_bag_name("/tmp/calibration_data.bag");
    if (argc > 2)
        data_bag_name = argv[2];
    ROS_INFO_STREAM("Loading calibration data from " << data_bag_name);

    if (!joint_calibration::loadBag(data_bag_name, description_msg, data)) {
        // Error will have been printed in function
        return -1;
    }


    // The parameter values
    joint_calibration::ParameterManager param_manager;

    // Load initial parameter values from YAML
    std::string param_file_name("/tmp/initial_params.yaml");
    if (argc > 4)
        param_file_name = argv[4];
    ROS_INFO_STREAM("Loading calibration data from " << param_file_name);

    if (!param_manager.loadFromYAML(param_file_name)) {
        // Error will have been printed in function
        return -1;
    }


    // Set up model
    // TODO: Remove hardcode here - use ROS parameters in launch file
    // TODO: Implement project function, make sure it's callable from Optimizer

    joint_calibration::ChainModel model(description_msg.data, "base_link", "fisheye");


    // Setup optimizer
    joint_calibration::Optimizer opt;
    // Run optimization
    opt.optimize(param_manager, data, model);
//
//    std::cout << params(0) << std::endl;
//    std::cout << params(17) << std::endl;
//
//    std::cout << param_manager.get("forearm_pitch_scaling") << std::endl;
//    std::cout << param_manager.get("fisheye_mount_c") << std::endl;
}
