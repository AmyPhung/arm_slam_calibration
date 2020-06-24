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

        /**
          *  \brief Load parameters and initial values from yaml file
          *  \param filename The name of the filename
          */
        bool loadFromYAML(std::string& filename);

        /**
         *  \brief Get value of parameter. Returns 0 if parameter not set.
         *  \param name The name of the parameter
         */
        double get(const std::string name);

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

    private:
        int num_free_params_;
        std::vector<std::string> param_order_;
        std::map<std::string, double> param_lookup_;

    };
}

#endif
