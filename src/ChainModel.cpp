//
// Created by amy on 6/23/20.
//

#include <joint_calibration/ChainModel.h>
#include <kdl_parser/kdl_parser.hpp>

namespace joint_calibration {

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

    void ChainModel::project(joint_calibration::ParameterManager &param_manager,
                             const joint_calibration::PointGroup &input_pts,
                             sensor_msgs::PointCloud &output_pts) {


        // Project each individual point
        for (size_t i = 0; i < input_pts.num_pts; ++i)
        {
            // Get the projection from forward kinematics of the robot chain
            // TODO: find a way to avoid re-computing this over and over - add to param manager?
            KDL::Frame fk = getChainFK(param_manager, input_pts.joint_states[i]);

//            std::cout<<fk.p.x()<<std::endl;
//            std::cout<<fk.p.y()<<std::endl;
//            std::cout<<fk.p.z()<<std::endl;

//            points[i].header.frame_id = root_;  // fk returns point in root_ frame
//
//            KDL::Frame p(KDL::Frame::Identity());
//            p.p.x(data.observations[sensor_idx].features[i].point.x);
//            p.p.y(data.observations[sensor_idx].features[i].point.y);
//            p.p.z(data.observations[sensor_idx].features[i].point.z);
//
//            // This is primarily for the case of checkerboards
//            //   The observation is in "checkerboard" frame, but the tip of the
//            //   kinematic chain is typically something like "wrist_roll_link".
//            if (data.observations[sensor_idx].features[i].header.frame_id != tip_)
//            {
//                KDL::Frame p2(KDL::Frame::Identity());
//                if (offsets.getFrame(data.observations[sensor_idx].features[i].header.frame_id, p2))
//                {
//                    // We have to apply the frame offset before the FK projection
//                    p = p2 * p;
//                }
//            }
//
//            // Apply the FK projection
//            p = fk * p;
//
//            points[i].point.x = p.p.x();
//            points[i].point.y = p.p.y();
//            points[i].point.z = p.p.z();
        }
//
//        return points;
//    }

    }

    KDL::Frame ChainModel::getChainFK(joint_calibration::ParameterManager& param_manager,
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
                double scale = param_manager.get(name + "_scaling");
                if (scale == 0.0) {
                    std::cout << "Scaling not set for " << name << ". Defaulting to 1" << std::endl;
                    scale = 1.0;
                }

                // Load offset param, default to 0 if not set
                double offset = param_manager.get(name + "_offset");
                if (offset == 0.0) {
                    std::cout << "Offset not set for " << name << ". Defaulting to 0" << std::endl;
                }

                // Apply any joint angle calibration
                double p = positionFromMsg(name, state) * scale + offset;
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











//ChainModel::ChainModel(const std::string& name, KDL::Tree model, std::string root, std::string tip) :
//        root_(root), tip_(tip), name_(name)
//{
//    // Create a KDL::Chain
//    if (!model.getChain(root, tip, chain_))
//        std::cerr << "Failed to get chain" << std::endl;
//}
//
//std::vector<geometry_msgs::PointStamped> ChainModel::project(
//        const robot_calibration_msgs::CalibrationData& data,
//        const CalibrationOffsetParser& offsets)
//{
//    // Projected points, to be returned
//    std::vector<geometry_msgs::PointStamped> points;
//
//    // Determine which observation to use
//    int sensor_idx = -1;
//    for (size_t obs = 0; obs < data.observations.size(); obs++)
//    {
//        if (data.observations[obs].sensor_name == name_)
//        {
//            sensor_idx = obs;
//            break;
//        }
//    }
//
//    if (sensor_idx < 0)
//    {
//        // TODO: any sort of error message?
//        return points;
//    }
//
//    // Resize to match # of features
//    points.resize(data.observations[sensor_idx].features.size());
//
//    // Get the projection from forward kinematics of the robot chain
//    KDL::Frame fk = getChainFK(offsets, data.joint_states);
//
//    // Project each individual point
//    for (size_t i = 0; i < points.size(); ++i)
//    {
//        points[i].header.frame_id = root_;  // fk returns point in root_ frame
//
//        KDL::Frame p(KDL::Frame::Identity());
//        p.p.x(data.observations[sensor_idx].features[i].point.x);
//        p.p.y(data.observations[sensor_idx].features[i].point.y);
//        p.p.z(data.observations[sensor_idx].features[i].point.z);
//
//        // This is primarily for the case of checkerboards
//        //   The observation is in "checkerboard" frame, but the tip of the
//        //   kinematic chain is typically something like "wrist_roll_link".
//        if (data.observations[sensor_idx].features[i].header.frame_id != tip_)
//        {
//            KDL::Frame p2(KDL::Frame::Identity());
//            if (offsets.getFrame(data.observations[sensor_idx].features[i].header.frame_id, p2))
//            {
//                // We have to apply the frame offset before the FK projection
//                p = p2 * p;
//            }
//        }
//
//        // Apply the FK projection
//        p = fk * p;
//
//        points[i].point.x = p.p.x();
//        points[i].point.y = p.p.y();
//        points[i].point.z = p.p.z();
//    }
//
//    return points;
//}
//

//}