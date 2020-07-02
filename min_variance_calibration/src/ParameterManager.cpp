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
