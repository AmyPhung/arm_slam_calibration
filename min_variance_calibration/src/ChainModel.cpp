//
// Created by amy on 6/23/20.
// Based on https://github.com/mikeferguson/robot_calibration/
//

#include <min_variance_calibration/ChainModel.h>

namespace min_variance_calibration {

    double positionFromMsg(const std::string& name,
                           const sensor_msgs::JointState& msg) {
        for (size_t i = 0; i < msg.name.size(); ++i) {
            if (msg.name[i] == name)
                return msg.position[i];
        }
        std::cerr << "Unable to find " << name << " in sensor_msgs::JointState" << std::endl;
        return 0.0;
    }

    ChainModel::ChainModel(const std::string &robot_description, std::string root, std::string tip) :
            root_(root), tip_(tip) {

        if (!model_.initString(robot_description)) {
            std::cerr << "Failed to parse URDF." << std::endl;
        }
        if (!kdl_parser::treeFromUrdfModel(model_, tree_)) {
            std::cerr << "Failed to construct KDL tree" << std::endl;
        }
        if (!tree_.getChain(root, tip, chain_)) {
            std::cerr << "Failed to get chain" << std::endl;
        }
    }

    ChainModel::~ChainModel() {
    }

    void ChainModel::project(min_variance_calibration::ParameterManager &param_manager,
                             const min_variance_calibration_msgs::PointGroup &input_pts,
                             sensor_msgs::PointCloud &output_pts) {

        // Output pointcloud will be in root frame
        output_pts.header.frame_id = root_;

        // Project each individual point
        for (size_t i = 0; i < input_pts.num_pts; ++i) {
            // Get the projection from forward kinematics of the robot chain
            // TODO: find a way to avoid re-computing this over and over - add to param manager?
            KDL::Frame fk = getChainFK(param_manager, input_pts.joint_states[i]);

            // Format observation as KDL Frame
            KDL::Frame p(KDL::Frame::Identity());
            p.p.x(input_pts.observations[i].point.x);
            p.p.y(input_pts.observations[i].point.y);
            p.p.z(input_pts.observations[i].point.z);

            // Apply the FK projection
            p = fk * p;

            // Create new point
            geometry_msgs::Point32 new_pt;

            // Fill in new point with computed projection (in root frame)
            new_pt.x = p.p.x();
            new_pt.y = p.p.y();
            new_pt.z = p.p.z();

            // Add point to pointcloud
            output_pts.points.push_back(new_pt);
        }
    }

    KDL::Frame ChainModel::getChainFK(min_variance_calibration::ParameterManager& param_manager,
                                      const sensor_msgs::JointState& state) {

        // FK from root to tip
        KDL::Frame p_out = KDL::Frame::Identity();

        // Step through joints to update p_out
        for (size_t i = 0; i < chain_.getNrOfSegments(); ++i) {
            std::string name = chain_.getSegment(i).getJoint().getName();

            // Get current correction value if current joint is a free frame
            KDL::Frame correction = KDL::Frame::Identity();
            param_manager.getFrame(name, correction);

            KDL::Frame pose;

            if (chain_.getSegment(i).getJoint().getType() != KDL::Joint::None) {
                // Load scaling param, default to 1 if not set
                double scale = param_manager.get(name + "_scaling") /
                        param_manager.getScale(name + "_scaling");
                if (scale == 0.0) {
//                    std::cout << "Scaling not set for " << name << ". Defaulting to 1" << std::endl;
                    scale = 1.0;
                }

                // Load offset param, default to 0 if not set
                double offset = param_manager.get(name + "_offset") /
                        param_manager.getScale(name + "_offset");
                if (offset == 0.0) {
//                    std::cout << "Offset not set for " << name << ". Defaulting to 0" << std::endl;
                }

                // Apply any joint angle calibration
                double p = ((positionFromMsg(name, state) - offset) / scale) * (M_PI / 180);

                // Get pose w.r.t. current joint at p joint angle
                pose = chain_.getSegment(i).pose(p);
            } else {
                // Get pose w.r.t. current joint assuming no adjustments
                pose = chain_.getSegment(i).pose(0.0);
            }

            KDL::Frame totip = chain_.getSegment(i).getFrameToTip();

            // Apply any frame calibration on the joint <origin> frame
            p_out = p_out * KDL::Frame(pose.p + totip.M * correction.p);
            p_out = p_out * KDL::Frame(totip.M * correction.M * totip.M.Inverse() * pose.M);
        }

        return p_out;
    }
}
