//
// Created by amy on 6/24/20.
//

#ifndef JOINT_CALIBRATION_PARAMETER_MANAGER_H
#define JOINT_CALIBRATION_PARAMETER_MANAGER_H

#include <kdl_parser/kdl_parser.hpp>
#include <joint_calibration/Utils.h>
#include <string>
#include <map>

namespace joint_calibration {
    class ParameterManager {
    public:
        ParameterManager();
        virtual ~ParameterManager();

        bool loadFromYAML(std::string& filename);
        double get(const std::string name);
        bool getFrame(const std::string name, KDL::Frame& offset);
        bool update(const ColumnVector& params);

    private:
        int num_free_params_;
        std::vector<std::string> param_order_;
        std::map<std::string, double> param_lookup_;


        /*
         * TODO:
         *  Parameter manager should load initial values from yaml,
         *  update using ColumnVector
         */
//
//        /** \brief Initialize the free_params */
//        bool initialize(double* free_params);
//
//        /**
//          *  \brief Tell the parser we wish to calibrate an active joint or other
//          *         single parameter.
//          *  \param name The name of the joint, e.g. "shoulder_pan_joint"
//          */
//        bool add(const std::string name);
//
//        /**
//         *  \brief Tell the parser we wish to calibrate a fixed joint.
//         *  \param name The name of the fixed joint, e.g. "head_camera_rgb_joint"
//         */
//        bool addFrame(const std::string name,
//                      bool calibrate_x, bool calibrate_y, bool calibrate_z,
//                      bool calibrate_roll, bool calibrate_pitch, bool calibrate_yaw);
//
//        /**
//         *  \brief Set the values for a single parameter.
//         *  \param name The name of the joint, e.g. "shoulder_pan_joint"
//         */
//        bool set(const std::string name, double value);
//
//        /**
//         *  \brief Set the values for a frame.
//         *  \param name The name of the fixed joint, e.g. "head_camera_rgb_joint"
//         */
//        bool setFrame(const std::string name,
//                      double x, double y, double z,
//                      double roll, double pitch, double yaw);
//
//        /** \brief Get a parameter value */
//        double get(const std::string name) const;
//
//        /**
//         *  \brief Get the offset for a frame calibration
//         *  \param name The name of the fixed joint, e.g. "head_camera_rgb_joint"
//         *  \param offset The KDL::Frame to fill in the offset.
//         *  \returns True if there is an offset to apply, false if otherwise.
//         */
//        bool getFrame(const std::string name, KDL::Frame& offset) const;
//
//    private:
//        // Names of parameters being calibrated. The order of this vector
//        // is the same as the free_param order will be interpreted.
//        std::vector<std::string> parameter_names_;
//
//        // Names of frames being calibrated.
//        std::vector<std::string> frame_names_;
//
//        // Values of parameters
//        std::vector<double> parameter_values_;
//
//        // Number of params being calibrated
//        size_t num_free_params_;
    };
}

#endif
