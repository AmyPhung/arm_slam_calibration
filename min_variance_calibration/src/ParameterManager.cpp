//
// Created by amy on 6/24/20.
// Based on https://github.com/mikeferguson/robot_calibration/
//


#include <min_variance_calibration/ParameterManager.h>
#include <fstream>


namespace min_variance_calibration {
    ParameterManager::ParameterManager() {
        num_free_params = 0;
    }

    ParameterManager::~ParameterManager() {
    }

    bool ParameterManager::loadFromYAML(std::string& filename) {
        // TODO: Make this function easier to read

        std::string line;
        std::ifstream f(filename.c_str());
        std::string curr_param;
        std::vector<double> lower_limits_vec_;
        std::vector<double> upper_limits_vec_;

        while (std::getline(f, line)) {
            std::istringstream str(line.c_str());
            std::string param;
            double value;

            if (str >> param >> value) {
                // This line should include initial values or limits

                // Remove the ":"
                param.erase(param.size() - 1);
                // Display parameter values
                std::cout << "  " << param << ": " << value << std::endl;

                if (param == "initial_value") {
                    // Insert param-value pairs into lookup table
                    param_lookup_[curr_param] = value;
                } else if (param == "lower_limit") {
                    lower_limits_vec_.push_back(value);
                } else if (param == "upper_limit") {
                    upper_limits_vec_.push_back(value);
                } else {
                    std::cerr << "Unrecognized parameter in yaml file" << std::endl;
                    return false;
                }

            } else {
                // This line should include the next parameter name
                str >> param;

                // Ignore blank lines
                if (param.empty()) {
                    continue;
                }

                // Remove the ":"
                param.erase(param.size() - 1);

                curr_param = param;
                param_order_.push_back(curr_param);
                num_free_params++;

                // Display parameter values
                std::cout << "Loading '" << curr_param << "' with values: " << std::endl;
            }
        }

        f.close();

        // Reformat upper and lower limits as column vector
        lower_limits = new ColumnVector(num_free_params);
        upper_limits = new ColumnVector(num_free_params);

        for (int i=0; i<num_free_params; i++) {
            lower_limits->operator()(i) = lower_limits_vec_[i];
            upper_limits->operator()(i) = upper_limits_vec_[i];
        }

        return true;
    }

    bool ParameterManager::loadFromROS(ros::NodeHandle& nh) {
        nh.param<int>("opt_npt", opt_npt, num_free_params + 2);
        nh.param<double>("opt_rho_begin", opt_rho_begin, 10);
        nh.param<double>("opt_rho_end", opt_rho_end, 1e-6);
        nh.param<int>("opt_max_f_evals", opt_max_f_evals, 10000);

        std::cout << "Loading 'opt_npt' with value " << opt_npt << std::endl;
        std::cout << "Loading 'opt_rho_begin' with value " << opt_rho_begin << std::endl;
        std::cout << "Loading 'opt_rho_end' with value " << opt_rho_end << std::endl;
        std::cout << "Loading 'opt_max_f_evals' with value " << opt_max_f_evals << std::endl;
        return true;
    }

    bool ParameterManager::loadFromMsg(const min_variance_calibration_msgs::RunCalibration::Request &req) {
        // Load Optimization Parameters
        opt_npt = req.opt_params.npt;
        opt_rho_begin = req.opt_params.rho_start;
        opt_rho_end = req.opt_params.rho_end;
        opt_max_f_evals = req.opt_params.max_f_evals;

        std::cout << "Loading 'opt_npt' with value " << opt_npt << std::endl;
        std::cout << "Loading 'opt_rho_begin' with value " << opt_rho_begin << std::endl;
        std::cout << "Loading 'opt_rho_end' with value " << opt_rho_end << std::endl;
        std::cout << "Loading 'opt_max_f_evals' with value " << opt_max_f_evals << std::endl;

        num_free_params = req.free_params.params.size();

        // Reformat upper and lower limits as column vector
        lower_limits = new ColumnVector(num_free_params);
        upper_limits = new ColumnVector(num_free_params);

        for (int i=0; i<num_free_params; i++) {
            std::cout << req.free_params.params[i].name <<std::endl;
            param_order_.push_back(req.free_params.params[i].name);
            param_lookup_[req.free_params.params[i].name] = req.free_params.params[i].value;
            scale_lookup_[req.free_params.params[i].name] = req.free_params.params[i].scaling;
            lower_limits->operator()(i) = req.free_params.params[i].min;
            upper_limits->operator()(i) = req.free_params.params[i].max;
        }

        return true;

    }

    double ParameterManager::get(const std::string name) {
        // Will return 0 if parameter not found
        return param_lookup_[name];
    }

    double ParameterManager::getScale(const std::string name) {
        // Will return 0 if parameter not found
        return scale_lookup_[name];
    }

    bool ParameterManager::getFrame(const std::string name,
            KDL::Frame& output_frame) {
        // TODO: Add scaling term here
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

    bool ParameterManager::getFreeParameters(min_variance_calibration_msgs::FreeParameters& free_params) {
        min_variance_calibration_msgs::ParameterInfo param_info;

        for (int i=0; i<num_free_params; i++) {
            // Rescale values to original units
            param_info.name = param_order_[i];
            param_info.min = lower_limits->operator()(i) / scale_lookup_[param_info.name];
            param_info.max = upper_limits->operator()(i) / scale_lookup_[param_info.name];
            param_info.value = param_lookup_[param_info.name] / scale_lookup_[param_info.name];

            // Restore to original scale
            param_info.scaling = 1;

            free_params.params.push_back(param_info);
        }
        return true;
    }

}
