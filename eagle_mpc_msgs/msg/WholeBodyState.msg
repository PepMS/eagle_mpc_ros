Header header

# This represent the time state
float64 time

# This represents the base state
eagle_mpc_msgs/FloatingBaseState floating_base

eagle_mpc_msgs/JointState[] joints

eagle_mpc_msgs/Thrust[] thrusts

# This represents the joint state of the arm (position, velocity, acceleration and effort)
# eagle_mpc_msgs/JointState[] joints
