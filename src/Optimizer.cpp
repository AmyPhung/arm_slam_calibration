//
// Created by amy on 6/23/20.
//

#include <joint_calibration/Optimizer.h>

#include <dlib/optimization.h>
#include <dlib/global_optimization.h>
#include <iostream>
#include <joint_calibration/PointGroup.h>
#include <boost/foreach.hpp> // Iterator

//using namespace std;
using namespace dlib;

namespace joint_calibration {

    double getPointComponent(sensor_msgs::PointCloud &input_pts, int pt, int component) {
        if (component == 0) {
            return input_pts.points[pt].x;
        } else if (component == 1) {
            return input_pts.points[pt].y;
        } else if (component == 2) {
            return input_pts.points[pt].z;
        } else {
            std::cout << "Desired Component does not exist" << std::endl;
            return 0.0;
        }
    }

    void computeCovarianceMatrix(sensor_msgs::PointCloud &input_pts,
                                 double (&covariance_matrix)[3][3]) {
        // Based on https://gist.github.com/atandrau/847214
        double means[3] = {0, 0, 0};
        for (int i = 0; i < input_pts.points.size(); i++)
            means[0] += input_pts.points[i].x,
            means[1] += input_pts.points[i].y,
            means[2] += input_pts.points[i].z;
        means[0] /= input_pts.points.size(),
        means[1] /= input_pts.points.size(),
        means[2] /= input_pts.points.size();

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) {
                covariance_matrix[i][j] = 0.0;
                for (int k = 0; k < input_pts.points.size(); k++)
                    covariance_matrix[i][j] += (means[i] - getPointComponent(input_pts, k, i)) *
                                               (means[j] - getPointComponent(input_pts, k, j));
                covariance_matrix[i][j] /= input_pts.points.size() - 1;
            }
    }

    double computeEigenvectorSum(double (&covariance_matrix)[3][3]) {
        // Sum of eigen values of a matrix is equal to the trace of a matrix
        double sum = 0;
        for (int i = 0; i < 3; i++)
            sum += (covariance_matrix[i][i]);
        return sum;
    }

    Optimizer::Optimizer() {
    }

    Optimizer::~Optimizer() {
    }

    void Optimizer::optimize(joint_calibration::ParameterManager& param_manager,
                             joint_calibration::CalibrationData& data,
                             joint_calibration::ChainModel& model) {

        // Reformat initial parameters
        ColumnVector initial_params(param_manager.num_free_params);
        param_manager.getColumnVector(initial_params);

        // Count iterations
        int iter = 0;
        auto objective_func = [&](const ColumnVector& params) {
            if (++iter%100 == 0)
                std::cout << "Iteration: " << iter << std::endl;
            param_manager.update(params);

            double variance_estimate = 0;

            // Iterate through point groups
            BOOST_FOREACH (joint_calibration::PointGroup const pts, data.point_groups) {
                sensor_msgs::PointCloud tf_points; // TODO: Init number of pts here
                model.project(param_manager, pts, tf_points);
                double covariance_matrix[3][3];
                computeCovarianceMatrix(tf_points, covariance_matrix);
                variance_estimate += computeEigenvectorSum(covariance_matrix);
            }

            return variance_estimate;
        };

        find_min_bobyqa(objective_func,
                        initial_params,
                        20,    // number of interpolation points TODO: optimize this
                        uniform_matrix<double>(param_manager.num_free_params,1, -1e100),  // lower bound constraint
                        uniform_matrix<double>(param_manager.num_free_params,1, 1e100),   // upper bound constraint
                        10,    // initial trust region radius
                        1e-6,  // stopping trust region radius
                        10000    // max number of objective function evaluations
        );

        // Display final parameters
        ColumnVector final_params(param_manager.num_free_params);
        param_manager.getColumnVector(final_params);

        std::cout << "Solution:\n" << final_params << std::endl;
        std::cout << "Objective Function End: " << objective_func(final_params) << std::endl;
    }
}
