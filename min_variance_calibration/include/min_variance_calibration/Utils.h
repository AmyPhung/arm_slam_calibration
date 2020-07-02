#ifndef MIN_VARIANCE_CALIBRATION_UTILS_H
#define MIN_VARIANCE_CALIBRATION_UTILS_H

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/query.h>
#include <boost/foreach.hpp>  // for rosbag iterator

#include <std_msgs/String.h>
#include <min_variance_calibration_msgs/CalibrationData.h>

// For rotation
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>

// For column vector
#include <dlib/optimization.h>

// For param loading
#include <iostream>
#include <iomanip>
#include <fstream>

namespace min_variance_calibration {

    typedef dlib::matrix<double, 0, 1> ColumnVector;

    namespace Utils {

        /**
         *  \brief Convert a vector containing parameter values to a ColumnVector
         *  \param initial_params Vector containing parameter values
         *  \param params This ColumnVector will be loaded with parameter values.
         */
        void reformatParams(std::vector<double> &initial_params,
                            min_variance_calibration::ColumnVector &params);

        KDL::Rotation rotationFromAxisMagnitude(const double x,
                                                const double y, const double z);

    }
}

#endif
