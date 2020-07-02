//
// Created by amy on 6/23/20.
// Based on https://github.com/mikeferguson/robot_calibration/
//

#include <ros/ros.h>
// ROS Messages
#include <std_msgs/String.h>

 // Custom Libraries
 #include <min_variance_calibration/ParameterManager.h>
 #include <min_variance_calibration/Utils.h>
 #include <min_variance_calibration/Optimizer.h>
 #include <min_variance_calibration/ChainModel.h>

// Calibration service
#include <min_variance_calibration_msgs/RunCalibration.h>

bool run_calibration(min_variance_calibration_msgs::RunCalibration::Request  &req,
                     min_variance_calibration_msgs::RunCalibration::Response &res) {

    ROS_INFO("Calling run_calibration service....");

    // Extract info from request
    std_msgs::String description_msg = req.robot_description;
    min_variance_calibration_msgs::CalibrationData data = req.data;
    min_variance_calibration::ParameterManager param_manager;
    param_manager.loadFromMsg(req);

    // TODO: remove hardcode root and tip
    // Set up chain model
    min_variance_calibration::ChainModel model(description_msg.data, "base_link", "fisheye");
    // Set up optimizer
    min_variance_calibration::Optimizer opt;
    // Run optimization
    opt.optimize(param_manager, data, model);

    return true;
}

int main(int argc, char** argv) {
    // ROS setup
    ros::init(argc, argv, "min_variance_calibration_server");
    ros::NodeHandle nh("~");

    ros::ServiceServer service = nh.advertiseService("/run_calibration", run_calibration);
    ROS_INFO("Ready to calibrate");
    ros::spin();

    return 0;
}


//
//    // The calibration data
//    std_msgs::String description_msg;
//    joint_calibration::CalibrationData data;
//
//    // Load calibration data from bagfile
//    std::string data_bag_name("/tmp/calibration_data.bag");
//    if (argc > 2)
//        data_bag_name = argv[2];
//    ROS_INFO_STREAM("Loading calibration data from " << data_bag_name);
//
//    if (!joint_calibration::Utils::loadBag(data_bag_name, description_msg, data)) {
//        // Error will have been printed in function
//        return -1;
//    }
//
//
//    // The parameter values
//    joint_calibration::ParameterManager param_manager;
//
//    // Load initial parameter values from YAML
//    std::string param_file_name("/tmp/initial_params.yaml");
//    if (argc > 4)
//        param_file_name = argv[4];
//    ROS_INFO_STREAM("Loading calibration data from " << param_file_name);
//
//    if (!param_manager.loadFromYAML(param_file_name)) {
//        // Error will have been printed in function
//        return -1;
//    }
//
//    // Load optimization parameters from ROS
//    if (!param_manager.loadFromROS(nh)) {
//        // Error will have been printed in function
//        return -1;
//    }
//
//
//    // Set up model
//    // TODO: Remove hardcode here - use ROS parameters in launch file
//    // TODO: Implement project function, make sure it's callable from Optimizer
//
//    joint_calibration::ChainModel model(description_msg.data, "base_link", "fisheye");
//
//
//    // Setup optimizer
//    joint_calibration::Optimizer opt;
//    // Run optimization
//    opt.optimize(param_manager, data, model);
////
////    std::cout << params(0) << std::endl;
////    std::cout << params(17) << std::endl;
////
////    std::cout << param_manager.get("forearm_pitch_scaling") << std::endl;
////    std::cout << param_manager.get("fisheye_mount_c") << std::endl;
