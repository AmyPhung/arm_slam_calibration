//
// Created by amy on 6/23/20.
//

#ifndef JOINT_CALIBRATION_OPTIMIZER_H
#define JOINT_CALIBRATION_OPTIMIZER_H

#include <joint_calibration/Utils.h>
#include <joint_calibration/CalibrationData.h>
#include <joint_calibration/ChainModel.h>

namespace joint_calibration {

    class Optimizer {
    public:
        Optimizer();
        virtual ~Optimizer();

        void optimize(ColumnVector& params,
                      std::vector<joint_calibration::CalibrationData>& data,
                      joint_calibration::ChainModel& model);

    private:
    };
}
#endif //JOINT_CALIBRATION_OPTIMIZER_H
