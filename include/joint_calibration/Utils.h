#ifndef JOINT_CALIBRATION_UTILS_H
#define JOINT_CALIBRATION_UTILS_H

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/query.h>
#include <boost/foreach.hpp>  // for rosbag iterator

#include <std_msgs/String.h>
#include <joint_calibration/CalibrationData.h>

namespace joint_calibration
{

/**
 *  \brief Load a bagfile of calibration data.
 *  \param file_name Name of the bag file to load.
 *  \param description_msg This will be loaded with the URDF string.
 *  \param data This will be loaded with the calibration data.
 */
    bool loadBag(const std::string& file_name,
                 std_msgs::String& description_msg,
                 std::vector<joint_calibration::CalibrationData>& data)
    {
        // Open the bag file
        rosbag::Bag bag_;
        try
        {
            bag_.open(file_name, rosbag::bagmode::Read);
        }
        catch (rosbag::BagException&)
        {
            ROS_FATAL_STREAM("Cannot open " << file_name);
            return false;
        }

        // Get robot_description from bag file
        rosbag::View model_view_(bag_, rosbag::TopicQuery("robot_description"));
        if (model_view_.size() < 1)
        {
            ROS_FATAL_STREAM("robot_description topic not found in bag file.");
            return false;
        }
        std_msgs::String::ConstPtr description_ = model_view_.begin()->instantiate<std_msgs::String>();
        description_msg = *description_;

        // Parse calibration_data topic
        rosbag::View data_view_(bag_, rosbag::TopicQuery("calibration_data"));
        BOOST_FOREACH (rosbag::MessageInstance const m, data_view_)
                    {
                        joint_calibration::CalibrationData::ConstPtr msg = m.instantiate<joint_calibration::CalibrationData>();
                        data.push_back(*msg);
                    }

        return true;
    }

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_LOAD_BAG_H
