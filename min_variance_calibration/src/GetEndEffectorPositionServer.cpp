//
// Created by amy on 6/23/20.
// Based on https://github.com/mikeferguson/robot_calibration/
//

#include <ros/ros.h>
// ROS Messages
#include <std_msgs/String.h>

// External Libraries
#include <boost/foreach.hpp> // Iterator

 // Custom Libraries
 #include <min_variance_calibration/ParameterManager.h>
 #include <min_variance_calibration/ChainModel.h>

// End Effector Position service
#include <min_variance_calibration_msgs/GetEndEffectorPosition.h>

bool get_end_effector_position(min_variance_calibration_msgs::GetEndEffectorPosition::Request  &req,
                               min_variance_calibration_msgs::GetEndEffectorPosition::Response &res) {

    ROS_INFO("Calling get_end_effector_position service....");

    // Extract info from request
    std::vector<sensor_msgs::JointState> joint_states = req.joint_states;
    min_variance_calibration_msgs::FreeParameters params = req.params;
    std_msgs::String description_msg = req.robot_description;
    std_msgs::String effector_frame = req.effector_frame;
    std_msgs::String output_frame = req.output_frame;

    // Set up chain model
    min_variance_calibration::ChainModel model(description_msg.data, output_frame.data, effector_frame.data);

    min_variance_calibration::ParameterManager param_manager;
    param_manager.loadFromMsg(req);

    res.output_poses.header.frame_id = output_frame.data;
    BOOST_FOREACH (sensor_msgs::JointState const state, joint_states) {
        KDL::Frame fk = model.getChainFK(param_manager, state);

        // Create new point
        geometry_msgs::Pose new_pt;
        new_pt.position.x = fk.p.x();
        new_pt.position.y = fk.p.y();
        new_pt.position.z = fk.p.z();

        fk.M.GetQuaternion(new_pt.orientation.x,
                           new_pt.orientation.y,
                           new_pt.orientation.z,
                           new_pt.orientation.w);

        // Add point to pointcloud
        res.output_poses.poses.push_back(new_pt);
    }

    ROS_INFO("get_end_effector_position service complete!");
    return true;
}

int main(int argc, char** argv) {
    // ROS setup
    ros::init(argc, argv, "min_variance_end_effector_position_server");
    ros::NodeHandle nh("~");

    // TODO: Add verbose mode for print statements
    ros::ServiceServer service = nh.advertiseService("/get_end_effector_position", get_end_effector_position);
    ROS_INFO("Ready to read end effector positions");
    ros::spin();

    return 0;
}
