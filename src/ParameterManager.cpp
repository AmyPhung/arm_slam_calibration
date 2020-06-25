//
// Created by amy on 6/24/20.
// Based on https://github.com/mikeferguson/robot_calibration/
//


#include <joint_calibration/ParameterManager.h>
#include <fstream>


namespace joint_calibration {
    ParameterManager::ParameterManager() {
        num_free_params = 0;
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
                num_free_params++;
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
        output_frame.p.x(get(std::string(name).append("_x_correction")));
        output_frame.p.y(get(std::string(name).append("_y_correction")));
        output_frame.p.z(get(std::string(name).append("_z_correction")));

        output_frame.M = Utils::rotationFromAxisMagnitude(
                get(std::string(name).append("_a_correction")),
                get(std::string(name).append("_b_correction")),
                get(std::string(name).append("_c_correction")));

        return true;
    }

    bool ParameterManager::update(const ColumnVector& params) {
        for (int i=0; i<num_free_params; i++){
            // Read input ColumnVector in order to update params
            const std::string &s = param_order_[i];
            param_lookup_[s] = params(i);
        }
        return true;
    }

    bool ParameterManager::getColumnVector(ColumnVector& output) {
        for (int i=0; i<num_free_params; i++){
            // Fill output ColumnVector in order
            const std::string &s = param_order_[i];
            output(i) = param_lookup_[s];
        }
        return true;
    }

}
