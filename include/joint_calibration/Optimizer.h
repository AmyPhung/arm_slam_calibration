//
// Created by amy on 6/23/20.
//

#ifndef JOINT_CALIBRATION_OPTIMIZER_H
#define JOINT_CALIBRATION_OPTIMIZER_H

#include <sensor_msgs/PointCloud.h>
#include <joint_calibration/CalibrationData.h>

#include "Utils.h"
#include "ChainModel.h"
#include "ParameterManager.h"

namespace joint_calibration {

    class Optimizer {
    public:
        Optimizer();
        virtual ~Optimizer();

        void optimize(joint_calibration::ParameterManager& param_manager,
                      joint_calibration::CalibrationData& data,
                      joint_calibration::ChainModel& model);

    private:
    };
}
#endif //JOINT_CALIBRATION_OPTIMIZER_H
