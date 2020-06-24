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

//            std::cout << line << std::endl;
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

//        // Print the final results.
//        for (int i = 0; i < param_order_.size(); ++i)
//        {
//            const std::string &s = param_order_[i];
//            std::cout << s << ' ' << param_lookup_[s] << '\n';
//        }

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
//        // Print the final results.
//        for (int i = 0; i < param_order_.size(); ++i)
//        {
//            const std::string &s = param_order_[i];
//            std::cout << s << ' ' << param_lookup_[s] << '\n';
//        }

//    bool ParameterManager::add(const std::string name) {
//        double value = 0.0;
//
//        // Check against parameters
//        for (size_t i = 0; i < parameter_names_.size(); ++i)
//        {
//            if (parameter_names_[i] == name)
//            {
//                if (i < num_free_params_)
//                {
//                    // This is already a free param, don't re-add
//                    return false;
//                }
//                // Get the current value
//                value = parameter_offsets_[i];
//                // Remove the non-free-param version
//                parameter_names_.erase(parameter_names_.begin() + i);
//                parameter_offsets_.erase(parameter_offsets_.begin() + i);
//            }
//        }
//
//        // Add the parameter at end of current free params
//        parameter_names_.insert(parameter_names_.begin() + num_free_params_, name);
//        parameter_offsets_.insert(parameter_offsets_.begin() + num_free_params_, value);
//        ++num_free_params_;
//        return true;
//    }
//
//
//   bool ParameterManager::loadOffsetYAML(const std::string& filename) {
//      std::string line;
//      std::ifstream f(filename.c_str());
//      while (std::getline(f, line))
//      {
//        std::istringstream str(line.c_str());
//        std::string param;
//        double value;
//        if (str >> param >> value)
//        {
//          // Remove the ":"
//          param.erase(param.size() - 1);
//          std::cout << "Loading '" << param << "' with value " << value << std::endl;
//          set(param, value);
//        }
//      }
//      f.close();
//      return true;
//    }
//
//
//
//     */
//    bool ParameterManager::set(const std::string name, double value)
//    {
//        for (size_t i = 0; i < num_free_params_; ++i)
//        {
//            if (parameter_names_[i] == name)
//            {
//                parameter_offsets_[i] = value;
//                return true;
//            }
//        }
//        return false;
//    }
//
//    double ParameterManager::get(const std::string name) {
//        // TODO: Replace this with ROS parameter names
//        for (size_t i = 0; i < parameter_names_.size(); ++i)
//        {
//            if (parameter_names_[i] == name)
//                return parameter_offsets_[i];
//        }
//        // Not calibrating this
//        return 0.0;
//    }
}
