#ifndef JOINT_CALIBRATION_UTILS_H
#define JOINT_CALIBRATION_UTILS_H

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/query.h>
#include <boost/foreach.hpp>  // for rosbag iterator

#include <std_msgs/String.h>
#include <joint_calibration/CalibrationData.h>

// For rotation
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>

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
                 joint_calibration::CalibrationData& data);

/**
 *  \brief Load a text file of initial parameter values. Ignores # lines
 *  \param file_name Name of the text file to load.
 *  \param params This will be loaded with parameter values.
 */
    bool loadParams(const std::string& file_name,
                          std::vector<double>& params);
/**
 *  \brief Convert a vector containing parameter values to a ColumnVector
 *  \param initial_params Vector containing parameter values
 *  \param params This ColumnVector will be loaded with parameter values.
 */
    void reformatParams(std::vector<double>& initial_params,
                        joint_calibration::ColumnVector& params);

    KDL::Rotation rotation_from_axis_magnitude(const double x,
            const double y, const double z);

}

#endif
