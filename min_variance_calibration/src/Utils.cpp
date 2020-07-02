//
// Created by amy on 6/24/20.
// Based on https://github.com/mikeferguson/robot_calibration/
//

#include <min_variance_calibration/Utils.h>

namespace min_variance_calibration {
    namespace Utils {

        void reformatParams(std::vector<double> &initial_params,
                            min_variance_calibration::ColumnVector &params) {
            for (int i = 0; i < initial_params.size(); i++) {
                params(i) = initial_params[i];
            }
        }

        KDL::Rotation rotationFromAxisMagnitude(const double x,
                                                const double y,
                                                const double z) {
            double magnitude = sqrt(x * x + y * y + z * z);

            if (magnitude == 0.0)
                return KDL::Rotation::Quaternion(0.0, 0.0, 0.0, 1.0);

            return KDL::Rotation::Quaternion(x / magnitude * sin(magnitude / 2.0),
                                             y / magnitude * sin(magnitude / 2.0),
                                             z / magnitude * sin(magnitude / 2.0),
                                             cos(magnitude / 2.0));
        }

    }
}
