//
// Created by amy on 6/24/20.
// Based on https://github.com/mikeferguson/robot_calibration/
//

#include <min_variance_calibration/Utils.h>

namespace min_variance_calibration {
    namespace Utils {

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
