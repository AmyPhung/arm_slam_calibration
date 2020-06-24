//
// Created by amy on 6/24/20.
//

#include <joint_calibration/Utils.h>

namespace joint_calibration {

    bool loadBag(const std::string& file_name,
                 std_msgs::String& description_msg,
                 joint_calibration::CalibrationData& data)
    {
        // Open the bag file
        rosbag::Bag bag_;
        try
        {
            bag_.open(file_name, rosbag::bagmode::Read);
        }
        catch (rosbag::BagException&)
        {
            ROS_FATAL_STREAM("Cannot open " << file_name);
            return false;
        }

        // Get robot_description from bag file
        rosbag::View model_view_(bag_, rosbag::TopicQuery("robot_description"));
        if (model_view_.size() < 1)
        {
            ROS_FATAL_STREAM("robot_description topic not found in bag file.");
            return false;
        }
        std_msgs::String::ConstPtr description_ = model_view_.begin()->instantiate<std_msgs::String>();
        description_msg = *description_;


        // Parse calibration_data topic
        rosbag::View data_view_(bag_, rosbag::TopicQuery("calibration_data"));
        BOOST_FOREACH (rosbag::MessageInstance const m, data_view_)
                    {
                        joint_calibration::CalibrationData::ConstPtr msg = m.instantiate<joint_calibration::CalibrationData>();
                        data = *msg;
                    }

        return true;
    }

    bool loadParams(const std::string& file_name,
                    std::vector<double>& params) {

        std::string file_line;
        std::ifstream param_file;

        param_file.open(file_name);
        if (!param_file) {
            ROS_FATAL_STREAM("Cannot open " << file_name);
            return false;
        }

        while (param_file >> file_line) {
            if (file_line[0] != '#') {
                double param_value = std::stof (file_line,nullptr);
                params.push_back(param_value);
            }
        }

        param_file.close();
        return true;
    }

    void reformatParams(std::vector<double>& initial_params,
                        joint_calibration::ColumnVector& params) {
        for (int i=0; i<initial_params.size(); i++) {
            params(i) = initial_params[i];
        }
    }

    KDL::Rotation rotation_from_axis_magnitude(const double x,
                                               const double y,
                                               const double z) {
        double magnitude = sqrt(x*x + y*y + z*z);

        if (magnitude == 0.0)
        return KDL::Rotation::Quaternion(0.0, 0.0, 0.0, 1.0);

        return KDL::Rotation::Quaternion(x/magnitude * sin(magnitude/2.0),
                y/magnitude * sin(magnitude/2.0),
                z/magnitude * sin(magnitude/2.0),
                cos(magnitude/2.0));
    }


}
