//
// Created by amy on 6/23/20.
//

#ifndef JOINT_CALIBRATION_CHAIN_MODEL_H
#define JOINT_CALIBRATION_CHAIN_MODEL_H


#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <urdf/model.h>
#include <joint_calibration/Utils.h>
#include <sensor_msgs/PointCloud.h>
#include <joint_calibration/PointGroup.h>
#include <joint_calibration/ParameterManager.h>

namespace joint_calibration {

    class ChainModel {
    public:
        ChainModel(const std::string& robot_description,
                std::string root, std::string tip);
        virtual ~ChainModel();
        void project(joint_calibration::ParameterManager& param_manager,
                const joint_calibration::PointGroup& input_pts,
                sensor_msgs::PointCloud& output_pts);

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
