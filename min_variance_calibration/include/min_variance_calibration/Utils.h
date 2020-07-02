#ifndef MIN_VARIANCE_CALIBRATION_UTILS_H
#define MIN_VARIANCE_CALIBRATION_UTILS_H

// For rotation
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>

// For column vector
#include <dlib/optimization.h>

namespace min_variance_calibration {

    typedef dlib::matrix<double, 0, 1> ColumnVector;

    namespace Utils {

        KDL::Rotation rotationFromAxisMagnitude(const double x,
                                                const double y, const double z);

    }
}

#endif
