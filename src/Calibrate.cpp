//
// Created by amy on 6/23/20.
//

#include <ros/ros.h>
// ROS Messages
#include <std_msgs/String.h>
#include <joint_calibration/CalibrationData.h>
// Custom Libraries
#include <joint_calibration/Utils.h>
#include <joint_calibration/Optimizer.h>
#include <joint_calibration/ChainModel.h>



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

    // The inital parameter values
    std::vector<double> initial_params;

    // Load initial parameter values from json
    std::string param_file_name("/tmp/initial_params.txt");
    if (argc > 4)
        param_file_name = argv[4];
    ROS_INFO_STREAM("Loading calibration data from " << param_file_name);

    if (!joint_calibration::loadParams(param_file_name, initial_params)) {
        // Error will have been printed in function
        return -1;
    }

    // Reformat as ColumnVector
    unsigned int num_params = initial_params.size();
    joint_calibration::ColumnVector params(num_params);
    joint_calibration::reformatParams(initial_params, params);

    // Set up model
    // TODO: Remove hardcode here
    joint_calibration::ChainModel model(description_msg.data, "base_link", "fisheye");
    // TODO: Implement project function, make sure it's callable from Optimizer

    // Setup optimizer
    joint_calibration::Optimizer opt;

    // Run optimization
    opt.optimize(params, data, model);

    std::cout << params(0) << std::endl;
    std::cout << params(17) << std::endl;
}
