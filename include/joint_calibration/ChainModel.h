//
// Created by amy on 6/23/20.
// Based on https://github.com/mikeferguson/robot_calibration/
//

#ifndef JOINT_CALIBRATION_CHAIN_MODEL_H
#define JOINT_CALIBRATION_CHAIN_MODEL_H


#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <urdf/model.h>
#include <joint_calibration/Utils.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/JointState.h>
#include <joint_calibration/PointGroup.h>
#include <joint_calibration/ParameterManager.h>

namespace joint_calibration {

    class ChainModel {
    public:
        ChainModel(const std::string& robot_description,
                std::string root, std::string tip);
        virtual ~ChainModel();

        /**
          *  \brief Compute point position along kinematic chain to project
          *  measurement taken in tip frame in root frame
          *  \param param_manager The parameter manager
          *  \param input_pts All of the observations of the same point
          *  \param output_pts The transformed observations written to a pointcloud
          */
        void project(joint_calibration::ParameterManager& param_manager,
                const joint_calibration::PointGroup& input_pts,
                sensor_msgs::PointCloud& output_pts);

        /**
          *  \brief Compute forward kinematics to get frame that transforms
          *  points to root frame
          *  \param param_manager The parameter manager
          *  \param state The joint states at the time of measurement
          */
        KDL::Frame getChainFK(joint_calibration::ParameterManager& param_manager,
                const sensor_msgs::JointState& state);

    private:
        urdf::Model model_;
        KDL::Tree tree_;
        KDL::Chain chain_;

    protected:
        std::string root_;
        std::string tip_;
    };

}


#endif
