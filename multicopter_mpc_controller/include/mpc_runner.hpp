#include <mutex>

#include <pinocchio/multibody/model.hpp>

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <ros/callback_queue.h>
#include <dynamic_reconfigure/server.h>

#include "nav_msgs/Odometry.h"
#include "mav_msgs/Actuators.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "visualization_msgs/Marker.h"
#include "multicopter_mpc_msgs/WholeBodyState.h"
#include "multicopter_mpc_msgs/MotorsState.h"
#include "multicopter_mpc_msgs/SolverPerformance.h"

#include "crocoddyl/core/utils/timer.hpp"
#include "crocoddyl/core/solvers/box-fddp.hpp"

#include "multicopter_mpc_controller/ParamsConfig.h"

#include "multicopter_mpc/trajectory.hpp"
#include "multicopter_mpc/mpc-controllers/carrot-mpc.hpp"
#include "multicopter_mpc/mpc-base.hpp"

class MpcRunner {
 public:
  MpcRunner();
  ~MpcRunner();

 private:
  void initializeParameters();
  void initializeMpcController();
  void initializeVariables();
  void initializeMsgs();
  void initializeSubscribers();
  void initializePublishers();

  void callbackOdometryMpc(const nav_msgs::OdometryConstPtr &msg_odometry);
  void callbackOdometryGains(const nav_msgs::OdometryConstPtr &msg_odometry);
  void callbackJointState(const sensor_msgs::JointStateConstPtr &msg_joint_state);
  void callbackConfig(multicopter_mpc_controller::ParamsConfig &config, uint32_t level);
  void callbackMpcSolve(const ros::TimerEvent &);

  // ROS objects & methods
  ros::NodeHandle nh_;

  ros::Subscriber subs_odom_;
  ros::Subscriber subs_joint_state_;

  ros::Publisher pub_thrust_command_;
  ros::Publisher pub_whole_body_state_;
  ros::Publisher pub_solver_performance_;
  std::vector<ros::Publisher> pub_arm_commands_;
  ros::Publisher pub_arm_command_;
  ros::Timer timer_mpc_solve_;

  dynamic_reconfigure::Server<multicopter_mpc_controller::ParamsConfig> server_;
  dynamic_reconfigure::Server<multicopter_mpc_controller::ParamsConfig>::CallbackType callback_server_;

  // Msgs
  mav_msgs::Actuators msg_thrusts_;
  multicopter_mpc_msgs::WholeBodyState msg_whole_body_state_;
  multicopter_mpc_msgs::SolverPerformance msg_solver_performance_;
  multicopter_mpc_msgs::MotorsState msg_motors_state_;
  std_msgs::Float64 msg_joint_command_;
  std::mutex mut_motors_state_;

  // Mpc Related
  boost::shared_ptr<multicopter_mpc::Trajectory> trajectory_;
  boost::shared_ptr<multicopter_mpc::CarrotMpc> carrot_mpc_;

  // Variables & Parameters
  struct NodeParams {
    std::string trajectory_config_path;
    std::size_t trajectory_dt;
    multicopter_mpc::SolverTypes trajectory_solver;
    std::string trajectory_integration;
    bool trajectory_squash;

    bool arm_enable;
    std::string arm_name;

    std::string mpc_config_path;
    std::string mpc_type;

    bool use_internal_gains;
    bool record_solver;
    std::size_t motor_command_dt;
  } node_params_;

  // MPC related
  double motor_command_dt_;

  // state
  std::mutex mut_state_;
  Eigen::VectorXd state_;
  Eigen::VectorXd state_ref_;
  Eigen::VectorXd state_diff_;
  ros::Time state_time_;

  // thrust command
  std::mutex mut_control_;
  Eigen::VectorXd control_command_;
  Eigen::VectorXd thrust_command_;
  // Eigen::VectorXd motors_thrust_new_;
  Eigen::VectorXd speed_command_;
  Eigen::VectorXd torque_command_;
  ros::Time control_last_;

  // gains
  std::mutex mut_gains_;
  bool flag_new_gains_;
  Eigen::MatrixXd fb_gains_;
  // Eigen::MatrixXd fb_gains_new_;

  // Solver
  ros::WallTime solver_time_init_;
  ros::WallDuration solver_duration_;

  // Controller state machine
  bool controller_started_;
  ros::Time controller_start_time_;
  ros::Duration controller_time_;
  std::size_t controller_instant_;
};

// #endif