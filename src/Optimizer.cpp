//
// Created by amy on 6/23/20.
//

#include <joint_calibration/Optimizer.h>

#include <dlib/optimization.h>
#include <dlib/global_optimization.h>
#include <iostream>


//using namespace std;
using namespace dlib;

namespace joint_calibration {
    Optimizer::Optimizer() {

    }

    Optimizer::~Optimizer() {
    }

}



//
//
//// ----------------------------------------------------------------------------------------
//
//// In dlib, most of the general purpose solvers optimize functions that take a
//// column vector as input and return a double.  So here we make a typedef for a
//// variable length column vector of doubles.  This is the type we will use to
//// represent the input to our objective functions which we will be minimizing.
//typedef matrix<double,0,1> column_vector;
//
//struct Point3D {
//    float x;
//    float y;
//    float z;
//};
//
//struct Group {
//
//};
//
//struct Observations {
//
//};
//
//int main() {
//    column_vector observations = {3, 5, 1, 7};
//
//    auto objective_func = [&](const column_vector& params) {
//
//        return mean(squared(params-observations));
//    };
//    column_vector initial_values = {-4,5,99,3};
//    find_min_bobyqa(objective_func,
//                    initial_values,
//                    9,    // number of interpolation points
//                    uniform_matrix<double>(4,1, -1e100),  // lower bound constraint
//                    uniform_matrix<double>(4,1, 1e100),   // upper bound constraint
//                    10,    // initial trust region radius
//                    1e-6,  // stopping trust region radius
//                    100    // max number of objective function evaluations
//    );
//    cout << "be_like_target solution:\n" << initial_values << endl;
//}
//
//// // https://gist.github.com/atandrau/847214
////void PointCloud::computeCovarianceMatrix() {
////    double means[3] = {0, 0, 0};
////    for (int i = 0; i < points.size(); i++)
////        means[0] += points[i].x,
////        means[1] += points[i].y,
////        means[2] += points[i].z;
////    means[0] /= points.size(), means[1] /= points.size(), means[2] /= points.size();
////
////    for (int i = 0; i < 3; i++)
////        for (int j = 0; j < 3; j++) {
////            covarianceMatrix[i][j] = 0.0;
////            for (int k = 0; k < points.size(); k++)
////                covarianceMatrix[i][j] += (means[i] - points[k].coord(i)) *
////                                          (means[j] - points[k].coord(j));
////            covarianceMatrix[i][j] /= points.size() - 1;
////        }
////}
//
//
