#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/JointState.h>
#include <apriltag_msgs/ApriltagArrayStamped.h>
#include <std_msgs/String.h>

#include <sstream>

using namespace sensor_msgs;
using namespace apriltag_msgs;
//using namespace message_filters;

void jsCallback(const JointState::ConstPtr& msg)
{
    ROS_INFO("asdfasfd");
}

void camCallback(const ApriltagArrayStamped::ConstPtr& msg)
{
    ROS_INFO("2222");
}

void callback(const JointState::ConstPtr& joint_states, const ApriltagArrayStamped::ConstPtr& tags)
{
    ROS_INFO("aaa");
    // Solve all of perception here...
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world ";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

}

int main(int argc, char** argv)
{
    ROS_INFO("hiya");
    ros::init(argc, argv, "calibration_node");
    ros::NodeHandle nh;

    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

//    ros::Subscriber sub = nh.subscribe("/joint_states", 1000, jsCallback);
//    ros::Subscriber sub2 = nh.subscribe("/camera/fisheye/tags", 1000, camCallback);
    message_filters::Subscriber<JointState> joint_sub(nh, "/joint_states", 1);
    message_filters::Subscriber<ApriltagArrayStamped> tag_sub(nh, "/camera/fisheye/tags", 1);

    typedef message_filters::sync_policies::ApproximateTime<JointState, ApriltagArrayStamped> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), joint_sub, tag_sub);

    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    return 0;
}