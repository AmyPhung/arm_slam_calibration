#ifndef JOINT_CALIBRATION_UTILS_H
#define JOINT_CALIBRATION_UTILS_H

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/query.h>
#include <boost/foreach.hpp>  // for rosbag iterator

#include <std_msgs/String.h>
#include <joint_calibration/CalibrationData.h>

// For column vector
#include <dlib/optimization.h>

// For param loading
#include <iostream>
#include <iomanip>
#include <fstream>

namespace joint_calibration
{
    typedef dlib::matrix<double,0,1> ColumnVector;

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


/**
 *  \brief Load a text file of initial parameter values. Ignores # lines
 *  \param file_name Name of the text file to load.
 *  \param params This will be loaded with parameter values.
 */
    bool loadParams(const std::string& file_name,
                          std::vector<double>& params) {

        std::string file_line;
        std::ifstream param_file;

        param_file.open(file_name);
        if (!param_file) {
            ROS_FATAL_STREAM("Cannot open " << file_name);
            return false;
        }

        while (param_file >> file_line) {
            if (file_line[0] != '#') {
                double param_value = std::stof (file_line,nullptr);
                params.push_back(param_value);
            }
        }

        param_file.close();
        return true;
    }

/**
 *  \brief Convert a vector containing parameter values to a ColumnVector
 *  \param initial_params Vector containing parameter values
 *  \param params This ColumnVector will be loaded with parameter values.
 */
    void reformatParams(std::vector<double>& initial_params,
                        joint_calibration::ColumnVector& params) {
        for (int i=0; i<initial_params.size(); i++) {
            params(i) = initial_params[i];
        }
    }

}

#endif
