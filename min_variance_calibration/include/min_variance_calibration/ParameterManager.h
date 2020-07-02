//
// Created by amy on 6/24/20.
// Based on https://github.com/mikeferguson/robot_calibration/
//

#ifndef MIN_VARIANCE_CALIBRATION_PARAMETER_MANAGER_H
#define MIN_VARIANCE_CALIBRATION_PARAMETER_MANAGER_H

#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <string>
#include <map>
#include <min_variance_calibration_msgs/RunCalibration.h>
#include <min_variance_calibration_msgs/FreeParameters.h>
#include <min_variance_calibration_msgs/ParameterInfo.h>

#include "Utils.h"

namespace min_variance_calibration {
    class ParameterManager {
    public:
        ParameterManager();
        virtual ~ParameterManager();

        /**
          *  \brief Load parameters and initial values from yaml file
          *  \param filename The name of the filename
          */
        bool loadFromYAML(std::string& filename);

        /**
          *  \brief Load optimization parameters from ROS/launch file
          *  \param nh ROS nodehandle
          */
        bool loadFromROS(ros::NodeHandle& nh);

        bool loadFromMsg(const min_variance_calibration_msgs::RunCalibration::Request &req);

        /**
         *  \brief Get value of parameter. Returns 0 if parameter not set.
         *  \param name The name of the parameter
         */
        double get(const std::string name);

        /**
         *  \brief Get scale of parameter.
         *  \param name The name of the parameter
         */
        double getScale(const std::string name);

        /**
         *  \brief Load frame parameters into KDL frame
         *  \param name The name of the frame to load
         *  \param output_frame Where the output frame will be saved to
         */
        bool getFrame(const std::string name, KDL::Frame& output_frame);

        /**
         *  \brief Update parameter manager based on values in ColumnVector
         *  \param params ColumnVector of parameters (in order)
         */
        bool update(const ColumnVector& params);

        /**
         *  \brief Output current parameters as a ColumnVector
         *  \param output ColumnVector containing current parameter values)
         */
        bool getColumnVector(ColumnVector& output);

        bool getFreeParameters(min_variance_calibration_msgs::FreeParameters& free_params);

        int num_free_params;
        int opt_npt;
        double opt_rho_begin;
        double opt_rho_end;
        int opt_max_f_evals;

        ColumnVector * lower_limits;
        ColumnVector * upper_limits;

    private:
        std::vector<std::string> param_order_;
        std::map<std::string, double> param_lookup_;
        std::map<std::string, double> scale_lookup_;

    };
}

#endif
