# This message describes the whole-body state of rigid body system.
#
# A whole-body state is defined by the centroidal, joints and contact states.
# The description of every element is defined in their respective
# messages. This state is described in respective time.
#
# The centroidal state describes 
#  * the position, velocity and acceleration of the CoM,
#  * the base orientation and angular velocity, and
#  * the linear and angular momentum and their rates.
#
# The joint states describes:
#  * the joint name
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s),
#  * the effort that is applied in the joint (Nm or N).
# The header specifies the time at which the state is published.

Header header

# This represent the time state
float64 time

# This represents the base state (CoM motion, angular motion and centroidal momenta)
eagle_mpc_msgs/FloatingBaseState floating_base

eagle_mpc_msgs/JointState[] joints

eagle_mpc_msgs/Thrust[] thrusts

# This represents the joint state of the arm (position, velocity, acceleration and effort)
# eagle_mpc_msgs/JointState[] joints