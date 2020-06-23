# Name of the landmark (should be unique for each)
string label

# Number of observations of this landmark
uint8 num_pts

# XYZ data at each observation
float32[] xs
float32[] ys
float32[] zs

# Joint state at each observation
sensor_msgs/JointState[] joint_states

