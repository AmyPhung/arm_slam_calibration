//
// Created by amy on 6/23/20.
//

#ifndef JOINT_CALIBRATION_CHAIN_MODEL_H
#define JOINT_CALIBRATION_CHAIN_MODEL_H


#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <urdf/model.h>

namespace joint_calibration {

    class ChainModel {
    public:
        ChainModel(const std::string& robot_description, std::string root, std::string tip);
        virtual ~ChainModel();

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
