//
// Created by amy on 6/23/20.
// Based on https://github.com/mikeferguson/robot_calibration/
//

#include <ros/ros.h>
// ROS Messages
#include <std_msgs/String.h>

 // Custom Libraries
 #include <min_variance_calibration/ParameterManager.h>
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
    opt.optimize(param_manager, data, model, res);

    ROS_INFO("Calibration Complete!");
    return true;
}

int main(int argc, char** argv) {
    // ROS setup
    ros::init(argc, argv, "min_variance_calibration_server");
    ros::NodeHandle nh("~");

    // TODO: Add verbose mode for print statements
    ros::ServiceServer service = nh.advertiseService("/run_calibration", run_calibration);
    ROS_INFO("Ready to calibrate");
    ros::spin();

    return 0;
}