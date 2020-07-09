//
// Created by amy on 7/8/20.
//

#include <ros/ros.h>

// Custom Libraries
#include <min_variance_calibration/ChainModel.h>

// Projection service
#include <min_variance_calibration_msgs/ProjectPoints.h>

bool project_points(min_variance_calibration_msgs::ProjectPoints::Request  &req,
                    min_variance_calibration_msgs::ProjectPoints::Response &res) {

    ROS_INFO("Calling project_points service....");

    // Extract info from request
    min_variance_calibration_msgs::CalibrationData data = req.input_data;
    min_variance_calibration_msgs::FreeParameters params = req.params;
    std_msgs::String description_msg = req.robot_description;
    std_msgs::String output_frame = req.output_frame;

    // TODO: remove hardcode root and tip
    // Set up chain model
    min_variance_calibration::ChainModel model(description_msg.data,
        "base_link", output_frame.data);
    min_variance_calibration::ParameterManager param_manager;
    param_manager.loadFromMsg(req);

    // TODO: remove hardcoded point group value
    model.project(param_manager, data.point_groups[0], res.output_points);
    ROS_INFO("Point projection Complete!");
    return true;
}

int main(int argc, char** argv) {
    // ROS setup
    ros::init(argc, argv, "point_projection_server");
    ros::NodeHandle nh("~");

    ros::ServiceServer service = nh.advertiseService("/project_points", project_points);
    ROS_INFO("Ready to project points");
    ros::spin();

    return 0;
}
