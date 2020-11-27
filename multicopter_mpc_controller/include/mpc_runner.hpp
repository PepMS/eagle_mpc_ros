#include <mutex>

#include <pinocchio/multibody/model.hpp>

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <ros/callback_queue.h>
#include <dynamic_reconfigure/server.h>

#include "nav_msgs/Odometry.h"
#include "mav_msgs/Actuators.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"
#include "multicopter_mpc_msgs/WholeBodyState.h"
#include "multicopter_mpc_msgs/MotorsState.h"
#include "multicopter_mpc_msgs/SolverPerformance.h"

#include "multicopter_mpc_controller/ParamsConfig.h"
#include "multicopter_mpc/mpc-main.hpp"

class MpcRunner {
 public:
  MpcRunner();
  ~MpcRunner();

  void runMpcStep();

 private:
  // ROS objects & methods
  ros::NodeHandle nh_;

  ros::Subscriber subs_odom_;
  ros::Subscriber subs_motors_speed_;

  ros::Publisher pub_motor_command_;
  ros::Publisher pub_trajectory_;
  ros::Publisher pub_whole_body_state_;
  ros::Publisher pub_motors_state_;
  ros::Publisher pub_solver_performance_;

  ros::Timer timer_motor_command_;
  ros::Timer timer_mpc_solve_;

  dynamic_reconfigure::Server<multicopter_mpc_controller::ParamsConfig> server_;
  dynamic_reconfigure::Server<multicopter_mpc_controller::ParamsConfig>::CallbackType callback_server_;

  mav_msgs::Actuators msg_actuators_;
  multicopter_mpc_msgs::WholeBodyState msg_whole_body_state_;
  multicopter_mpc_msgs::SolverPerformance msg_solver_performance_;

  void callbackOdometry(const nav_msgs::OdometryConstPtr &msg_odometry);
  void callbackMotorCommand(const ros::TimerEvent &);
  void callbackMotorCommandGains(const ros::TimerEvent &);
  void callbackMpcSolve(const ros::TimerEvent &);
  void callbackConfig(multicopter_mpc_controller::ParamsConfig &config, uint32_t level);
  void callbackMotorSpeed(const mav_msgs::ActuatorsConstPtr &msg_motor_speed);

  void callbackOdometryCascade(const nav_msgs::OdometryConstPtr &msg_odometry);

  void callbackPublishAndSolve(const ros::TimerEvent &);
  void callbackSolveAndPublish(const ros::TimerEvent &);

  // MPC related
  double motor_command_dt_;
  bool record_solver_;

  // state
  std::mutex mut_state_;
  Eigen::VectorXd state_;
  Eigen::VectorXd state_ref_;
  Eigen::VectorXd state_diff_;
  ros::Time state_time_;
  ros::Time state_used_time_;
  boost::shared_ptr<crocoddyl::StateMultibody> robot_state_;

  // thrust command
  std::mutex mut_control_;
  Eigen::VectorXd motors_thrust_;
  Eigen::VectorXd motors_thrust_new_;
  Eigen::VectorXd motors_speed_;
  ros::Time control_time_;

  // gains
  std::mutex mut_gains_;
  bool flag_new_gains_;
  Eigen::VectorXd ff_gains_;
  Eigen::MatrixXd fb_gains_;
  Eigen::MatrixXd fb_gains_new_;

  std::mutex mut_motors_state_;
  multicopter_mpc_msgs::MotorsState msg_motors_state_;

  multicopter_mpc::MpcMain mpc_main_;

  ros::Time time_1_;

  // Solver
  ros::WallTime solver_time_init_;
  ros::WallDuration solver_duration_;

  // Controller state machine
  bool controller_started_;

  // Visualization related
  visualization_msgs::Marker trajectory_points_;

  void fillTrajectoryMsg();

  // Debug
  bool printed_;
};

// #endif