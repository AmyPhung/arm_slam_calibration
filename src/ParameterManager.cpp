//
// Created by amy on 6/24/20.
// Based on https://github.com/mikeferguson/robot_calibration/
//


#include <joint_calibration/ParameterManager.h>
#include <fstream>


namespace joint_calibration {
    ParameterManager::ParameterManager() {
        num_free_params_ = 0;
    }

    ParameterManager::~ParameterManager() {
    }

    bool ParameterManager::loadFromYAML(std::string& filename) {

        std::string line;
        std::ifstream f(filename.c_str());
        while (std::getline(f, line)) {
            std::istringstream str(line.c_str());
            std::string param;
            double value;

            if (str >> param >> value) {
                // Remove the ":"
                param.erase(param.size() - 1);
                std::cout << "Loading '" << param << "' with value " << value << std::endl;

                // Insert param-value pairs into lookup table
                param_lookup_[param] = value;
                param_order_.push_back(param);
                num_free_params_++;
            }
        }

        f.close();

        return true;
    }

    double ParameterManager::get(const std::string name) {
        // Will return 0 if parameter not found
        return param_lookup_[name];
    }

    bool ParameterManager::getFrame(const std::string name,
            KDL::Frame& output_frame) {
        output_frame.p.x(get(std::string(name).append("_x")));
        output_frame.p.y(get(std::string(name).append("_y")));
        output_frame.p.z(get(std::string(name).append("_z")));

        output_frame.M = rotation_from_axis_magnitude(
                get(std::string(name).append("_a")),
                get(std::string(name).append("_b")),
                get(std::string(name).append("_c")));

        return true;
    }

    bool ParameterManager::update(const ColumnVector& params) {
        for (int i=0; i<num_free_params_; i++){
            const std::string &s = param_order_[i];
            param_lookup_[s] = params(i);
        }
        return true;
    }

}
