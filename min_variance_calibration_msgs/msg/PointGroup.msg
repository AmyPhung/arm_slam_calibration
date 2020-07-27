# Number of observations of this landmark
uint8 num_pts

geometry_msgs/PointStamped[] observations

# Joint state at each observation
sensor_msgs/JointState[] joint_states

# Known point position in root frame
geometry_msgs/PointStamped ground_truth
