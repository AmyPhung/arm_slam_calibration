//
// Created by amy on 6/23/20.
//

#include "ChainModel.h"


ChainModel::ChainModel(const std::string& name, KDL::Tree model, std::string root, std::string tip) :
        root_(root), tip_(tip), name_(name)
{
    // Create a KDL::Chain
    if (!model.getChain(root, tip, chain_))
        std::cerr << "Failed to get chain" << std::endl;
}

std::vector<geometry_msgs::PointStamped> ChainModel::project(
        const robot_calibration_msgs::CalibrationData& data,
        const CalibrationOffsetParser& offsets)
{
    // Projected points, to be returned
    std::vector<geometry_msgs::PointStamped> points;

    // Determine which observation to use
    int sensor_idx = -1;
    for (size_t obs = 0; obs < data.observations.size(); obs++)
    {
        if (data.observations[obs].sensor_name == name_)
        {
            sensor_idx = obs;
            break;
        }
    }

    if (sensor_idx < 0)
    {
        // TODO: any sort of error message?
        return points;
    }

    // Resize to match # of features
    points.resize(data.observations[sensor_idx].features.size());

    // Get the projection from forward kinematics of the robot chain
    KDL::Frame fk = getChainFK(offsets, data.joint_states);

    // Project each individual point
    for (size_t i = 0; i < points.size(); ++i)
    {
        points[i].header.frame_id = root_;  // fk returns point in root_ frame

        KDL::Frame p(KDL::Frame::Identity());
        p.p.x(data.observations[sensor_idx].features[i].point.x);
        p.p.y(data.observations[sensor_idx].features[i].point.y);
        p.p.z(data.observations[sensor_idx].features[i].point.z);

        // This is primarily for the case of checkerboards
        //   The observation is in "checkerboard" frame, but the tip of the
        //   kinematic chain is typically something like "wrist_roll_link".
        if (data.observations[sensor_idx].features[i].header.frame_id != tip_)
        {
            KDL::Frame p2(KDL::Frame::Identity());
            if (offsets.getFrame(data.observations[sensor_idx].features[i].header.frame_id, p2))
            {
                // We have to apply the frame offset before the FK projection
                p = p2 * p;
            }
        }

        // Apply the FK projection
        p = fk * p;

        points[i].point.x = p.p.x();
        points[i].point.y = p.p.y();
        points[i].point.z = p.p.z();
    }

    return points;
}

KDL::Frame ChainModel::getChainFK(const CalibrationOffsetParser& offsets,
                                  const sensor_msgs::JointState& state)
{
    // FK from root to tip
    KDL::Frame p_out = KDL::Frame::Identity();

    // Step through joints
    for (size_t i = 0; i < chain_.getNrOfSegments(); ++i)
    {
        std::string name = chain_.getSegment(i).getJoint().getName();
        KDL::Frame correction = KDL::Frame::Identity();
        offsets.getFrame(name, correction);

        KDL::Frame pose;
        if (chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
        {
            // Apply any joint offset calibration
            double p = positionFromMsg(name, state) + offsets.get(name);
            pose = chain_.getSegment(i).pose(p);
        }
        else
        {
            pose = chain_.getSegment(i).pose(0.0);
        }

        KDL::Frame totip = chain_.getSegment(i).getFrameToTip();

        // Apply any frame calibration on the joint <origin> frame
        p_out = p_out * KDL::Frame(pose.p + totip.M * correction.p);
        p_out = p_out * KDL::Frame(totip.M * correction.M * totip.M.Inverse() * pose.M);

    }
    return p_out;
}