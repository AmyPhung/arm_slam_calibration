//
// Created by amy on 6/23/20.
//

#ifndef MIN_VARIANCE_CALIBRATION_OPTIMIZER_H
#define MIN_VARIANCE_CALIBRATION_OPTIMIZER_H

#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PointStamped.h>
#include <min_variance_calibration_msgs/CalibrationData.h>
#include <min_variance_calibration_msgs/RunCalibration.h>

#include "Utils.h"
#include "ChainModel.h"
#include "ParameterManager.h"

namespace min_variance_calibration {

    class Optimizer {
    public:
        Optimizer();
        virtual ~Optimizer();

        void optimize(min_variance_calibration::ParameterManager& param_manager,
                      min_variance_calibration_msgs::CalibrationData& data,
                      min_variance_calibration::ChainModel& model,
                      min_variance_calibration_msgs::RunCalibration::Response &res);

    private:
    };
}
#endif
