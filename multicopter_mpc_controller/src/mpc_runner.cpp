#include "mpc_runner.hpp"

MpcRunner::MpcRunner() {
  std::string mission_path;
  std::string mpc_controller_type;
  std::string mpc_main_yaml_path;
  bool use_internal_gains;
  double time_step;

  // Parameters
  nh_.param<std::string>(ros::this_node::getNamespace() + "/mission_path", mission_path, "");

  nh_.param<bool>(ros::this_node::getNamespace() + "/use_internal_gains", use_internal_gains, "false");
  nh_.param<double>(ros::this_node::getNamespace() + "/motor_command_dt", motor_command_dt_, 0.01);

  if (!use_internal_gains) {
    mpc_main_yaml_path = "/home/pepms/wsros/mpc-ws/src/multicopter_mpc/multicopter_mpc_yaml/mpc_main/mpc-main.yaml";
  } else {
    ROS_INFO("Using internal gains");
    mpc_main_yaml_path = "/home/pepms/wsros/mpc-ws/src/multicopter_mpc/multicopter_mpc_yaml/mpc_main/mpc-main.yaml";
  }

  // Mpc Controller
  mpc_main_ = multicopter_mpc::MpcMain(multicopter_mpc::MultiCopterTypes::Iris, mission_path, mpc_main_yaml_path);

  // States
  state_ = mpc_main_.getMpcController()->getStateMultibody()->zero();
  state_ref_ = state_;
  state_diff_ = Eigen::VectorXd::Zero(mpc_main_.getMpcController()->getStateMultibody()->get_ndx());

  // Thrust command
  motors_thrust_ = Eigen::VectorXd::Zero(mpc_main_.getMpcController()->getActuation()->get_nu());
  motors_speed_ = motors_thrust_;

  // Gains
  flag_new_gains_ = false;
  fb_gains_ = Eigen::MatrixXd::Zero(mpc_main_.getMpcController()->getActuation()->get_nu(),
                                    mpc_main_.getMpcController()->getStateMultibody()->get_ndx());

  msg_actuators_.angular_velocities.resize(motors_thrust_.size());
  msg_motors_state_.thrusts.resize(motors_thrust_.size());
  msg_whole_body_state_.thrusts.resize(motors_thrust_.size());

  controller_started_ = false;

  // Subscribers
  // subs_odom_ = nh_.subscribe("/odometry", 1, &MpcRunner::callbackOdometry, this,
  // ros::TransportHints().tcpNoDelay());
  subs_odom_ =
      nh_.subscribe("/odometry", 1, &MpcRunner::callbackOdometryCascade, this, ros::TransportHints().tcpNoDelay());
  // subs_motors_speed_ =
  //     nh_.subscribe("/motors_speed", 1, &MpcRunner::callbackMotorSpeed, this, ros::TransportHints().tcpNoDelay());
  std::cout << "herer!!" << std::endl;

  // Publishers
  pub_motor_command_ = nh_.advertise<mav_msgs::Actuators>("/motor_speed", 1);
  pub_whole_body_state_ = nh_.advertise<multicopter_mpc_msgs::WholeBodyState>("/whole_body_state", 1);
  // pub_motors_state_ = nh_.advertise<multicopter_mpc_msgs::MotorsState>("/motors_state", 10);

  // Timers
  if (use_internal_gains) {
    // timer_motor_command_ =
    //     nh_.createTimer(ros::Duration(motor_command_dt_), &MpcRunner::callbackMotorCommandGains, this);
    // timer_mpc_solve_ = nh_.createTimer(ros::Duration(mpc_main_.getMpcController()->getTimeStep()),
    //                                    &MpcRunner::callbackMpcSolve, this);
    // ROS_INFO_STREAM("Publishing motor control every: " << motor_command_dt_ << " seconds");
  } else {
    // timer_motor_command_ = nh_.createTimer(ros::Duration(mpc_main_.getMpcController()->getTimeStep()),
    //                                        &MpcRunner::callbackSolveAndPublish, this);
    // ROS_INFO_STREAM("Publishing motor control every: " << motor_command_dt_ << " seconds");
  }

  // RQT Config
  callback_server_ = boost::bind(&MpcRunner::callbackConfig, this, _1, _2);
  server_.setCallback(callback_server_);

  printed_ = false;
}

MpcRunner::~MpcRunner() {}

void MpcRunner::callbackOdometryCascade(const nav_msgs::OdometryConstPtr &msg_odometry) {
  mut_state_.lock();
  state_(0) = msg_odometry->pose.pose.position.x;
  state_(1) = msg_odometry->pose.pose.position.y;
  state_(2) = msg_odometry->pose.pose.position.z;
  state_(3) = msg_odometry->pose.pose.orientation.x;
  state_(4) = msg_odometry->pose.pose.orientation.y;
  state_(5) = msg_odometry->pose.pose.orientation.z;
  state_(6) = msg_odometry->pose.pose.orientation.w;
  state_(7) = msg_odometry->twist.twist.linear.x;
  state_(8) = msg_odometry->twist.twist.linear.y;
  state_(9) = msg_odometry->twist.twist.linear.z;
  state_(10) = msg_odometry->twist.twist.angular.x;
  state_(11) = msg_odometry->twist.twist.angular.y;
  state_(12) = msg_odometry->twist.twist.angular.z;
  state_time_ = msg_odometry->header.stamp;
  mut_state_.unlock();

  if (controller_started_) {
    mut_state_.lock();
    mpc_main_.setCurrentState(state_);
    mut_state_.unlock();

    msg_whole_body_state_.header.stamp = ros::Time::now();
    msg_whole_body_state_.floating_base.pose = msg_odometry->pose.pose;
    msg_whole_body_state_.floating_base.motion = msg_odometry->twist.twist;

    mpc_main_.runMpcStep(1);
    motors_speed_ = mpc_main_.getMotorsSpeed();

    if (ros::Duration(ros::Time::now() - control_time_).toSec() > 1.5 * mpc_main_.getMpcController()->getTimeStep()) {
      ROS_WARN("Control rate not accomplished!");
    }
    msg_actuators_.header.stamp = ros::Time::now();
    msg_actuators_.angular_velocities[0] = motors_speed_(0);
    msg_actuators_.angular_velocities[1] = motors_speed_(1);
    msg_actuators_.angular_velocities[2] = motors_speed_(2);
    msg_actuators_.angular_velocities[3] = motors_speed_(3);
    pub_motor_command_.publish(msg_actuators_);

    for (std::size_t i = 0; i < 4; ++i) {
      msg_whole_body_state_.thrusts[i].speed_command = motors_speed_(i);
    }

    int cursor = mpc_main_.getCursor() - mpc_main_.getMpcController()->getKnots() - 1;
    if (cursor < int(mpc_main_.getMpcController()->getTrajectoryGenerator()->getKnots())) {
      pub_whole_body_state_.publish(msg_whole_body_state_);
    }

    control_time_ = ros::Time::now();
  }
}

void MpcRunner::callbackOdometry(const nav_msgs::OdometryConstPtr &msg_odometry) {
  mut_state_.lock();
  state_(0) = msg_odometry->pose.pose.position.x;
  state_(1) = msg_odometry->pose.pose.position.y;
  state_(2) = msg_odometry->pose.pose.position.z;
  state_(3) = msg_odometry->pose.pose.orientation.x;
  state_(4) = msg_odometry->pose.pose.orientation.y;
  state_(5) = msg_odometry->pose.pose.orientation.z;
  state_(6) = msg_odometry->pose.pose.orientation.w;
  state_(7) = msg_odometry->twist.twist.linear.x;
  state_(8) = msg_odometry->twist.twist.linear.y;
  state_(9) = msg_odometry->twist.twist.linear.z;
  state_(10) = msg_odometry->twist.twist.angular.x;
  state_(11) = msg_odometry->twist.twist.angular.y;
  state_(12) = msg_odometry->twist.twist.angular.z;
  state_time_ = msg_odometry->header.stamp;
  mut_state_.unlock();
}

void MpcRunner::callbackMotorSpeed(const mav_msgs::ActuatorsConstPtr &msg_motor_speed) {
  mut_motors_state_.lock();
  msg_motors_state_.header.stamp = ros::Time::now();
  msg_motors_state_.thrusts[0].speed_reading = msg_motor_speed->angular_velocities[0];
  msg_motors_state_.thrusts[1].speed_reading = msg_motor_speed->angular_velocities[1];
  msg_motors_state_.thrusts[2].speed_reading = -msg_motor_speed->angular_velocities[2];
  msg_motors_state_.thrusts[3].speed_reading = -msg_motor_speed->angular_velocities[3];
  pub_motors_state_.publish(msg_motors_state_);
  mut_motors_state_.unlock();
}

void MpcRunner::callbackMotorCommand(const ros::TimerEvent &) {
  if (controller_started_) {
    msg_actuators_.header.stamp = ros::Time::now();
    msg_actuators_.angular_velocities[0] = motors_speed_(0);
    msg_actuators_.angular_velocities[1] = motors_speed_(1);
    msg_actuators_.angular_velocities[2] = motors_speed_(2);
    msg_actuators_.angular_velocities[3] = motors_speed_(3);
    pub_motor_command_.publish(msg_actuators_);

    mut_state_.lock();
    mpc_main_.setCurrentState(state_);
    state_used_time_ = state_time_;
    ros::Duration elapsed = ros::Time::now() - state_time_;
    ROS_INFO("State use in control was %lf seconds before", elapsed.toSec());
    mut_state_.unlock();

    mpc_main_.runMpcStep();
    if (mpc_main_.getMpcController()->getSolver()->get_stop() > 1e-4) {
      ROS_WARN("Big stop value: %f", mpc_main_.getMpcController()->getSolver()->get_stop());
    }

    motors_speed_ = mpc_main_.getMotorsSpeed();
    if (ros::Duration(ros::Time::now() - control_time_).toSec() > 0.004) {
      ROS_WARN("Control rate not accomplished!");
    }

    control_time_ = ros::Time::now();
  }
}

void MpcRunner::callbackPublishAndSolve(const ros::TimerEvent &) {
  // equivalent to callbackMotorCommand
  if (controller_started_) {
    msg_actuators_.header.stamp = ros::Time::now();
    msg_actuators_.angular_velocities[0] = motors_speed_(0);
    msg_actuators_.angular_velocities[1] = motors_speed_(1);
    msg_actuators_.angular_velocities[2] = motors_speed_(2);
    msg_actuators_.angular_velocities[3] = motors_speed_(3);
    pub_motor_command_.publish(msg_actuators_);

    mut_state_.lock();
    mpc_main_.setCurrentState(state_);
    state_used_time_ = state_time_;
    ros::Duration elapsed = ros::Time::now() - state_time_;
    ROS_INFO("State use in control was %lf seconds before", elapsed.toSec());
    mut_state_.unlock();

    mpc_main_.runMpcStep();
    if (mpc_main_.getMpcController()->getSolver()->get_stop() > 1e-4) {
      ROS_WARN("Big stop value: %f", mpc_main_.getMpcController()->getSolver()->get_stop());
    }

    motors_speed_ = mpc_main_.getMotorsSpeed();
    if (ros::Duration(ros::Time::now() - control_time_).toSec() > 0.004) {
      ROS_WARN("Control rate not accomplished!");
    }

    control_time_ = ros::Time::now();
  }
}

void MpcRunner::callbackSolveAndPublish(const ros::TimerEvent &) {
  if (controller_started_) {
    mut_state_.lock();

    mpc_main_.setCurrentState(state_);
    state_used_time_ = state_time_;
    ros::Duration elapsed = ros::Time::now() - state_time_;
    msg_whole_body_state_.header.stamp = ros::Time::now();
    msg_whole_body_state_.floating_base.pose.position.x = state_(0);
    msg_whole_body_state_.floating_base.pose.position.y = state_(1);
    msg_whole_body_state_.floating_base.pose.position.z = state_(2);
    msg_whole_body_state_.floating_base.pose.orientation.x = state_(3);
    msg_whole_body_state_.floating_base.pose.orientation.y = state_(4);
    msg_whole_body_state_.floating_base.pose.orientation.z = state_(5);
    msg_whole_body_state_.floating_base.pose.orientation.w = state_(6);
    msg_whole_body_state_.floating_base.motion.linear.x = state_(7);
    msg_whole_body_state_.floating_base.motion.linear.y = state_(8);
    msg_whole_body_state_.floating_base.motion.linear.z = state_(9);
    msg_whole_body_state_.floating_base.motion.angular.x = state_(10);
    msg_whole_body_state_.floating_base.motion.angular.y = state_(11);
    msg_whole_body_state_.floating_base.motion.angular.z = state_(12);

    mut_state_.unlock();

    mpc_main_.runMpcStep();

    motors_speed_ = mpc_main_.getMotorsSpeed();
    if (ros::Duration(ros::Time::now() - control_time_).toSec() > 0.004) {
      ROS_WARN("Control rate not accomplished!");
    }
    msg_actuators_.header.stamp = ros::Time::now();
    msg_actuators_.angular_velocities[0] = motors_speed_(0);
    msg_actuators_.angular_velocities[1] = motors_speed_(1);
    msg_actuators_.angular_velocities[2] = motors_speed_(2);
    msg_actuators_.angular_velocities[3] = motors_speed_(3);
    pub_motor_command_.publish(msg_actuators_);

    // mut_motors_state_.lock();
    // msg_motors_state_.thrusts[0].speed_command = motors_speed_(0);
    // msg_motors_state_.thrusts[1].speed_command = motors_speed_(1);
    // msg_motors_state_.thrusts[2].speed_command = motors_speed_(2);
    // msg_motors_state_.thrusts[3].speed_command = motors_speed_(3);
    // mut_motors_state_.unlock();

    for (std::size_t i = 0; i < 4; ++i) {
      msg_whole_body_state_.thrusts[i].speed_command = motors_speed_(i);
    }

    int cursor = mpc_main_.getCursor() - mpc_main_.getMpcController()->getKnots() - 1;
    if (cursor < int(mpc_main_.getMpcController()->getTrajectoryGenerator()->getKnots())) {
      pub_whole_body_state_.publish(msg_whole_body_state_);
    }
    control_time_ = ros::Time::now();
  }
}

void MpcRunner::callbackMotorCommandGains(const ros::TimerEvent &) {
  if (controller_started_) {
    if (flag_new_gains_) {
      mut_gains_.lock();
      motors_thrust_ = motors_thrust_new_;
      fb_gains_ = fb_gains_new_;
      flag_new_gains_ = false;
      mut_gains_.unlock();
    }

    mut_state_.lock();
    ros::Duration elapsed_time = ros::Time::now() - control_time_;
    if (elapsed_time > ros::Duration(motor_command_dt_)) {
      // ROS_WARN("Motor commands: Outdated MPC solution: %f", elapsed_time.toSec());
    }
    mpc_main_.getMpcController()->getStateMultibody()->diff(state_ref_, state_, state_diff_);
    // robot_state_->diff(state_ref_, state_, state_diff_);
    mut_state_.unlock();

    mut_control_.lock();
    motors_thrust_ = motors_thrust_ - fb_gains_ * state_diff_;
    motors_thrust_ = (motors_thrust_.array() < 0).select(0, motors_thrust_);  // Avoid negative numbers
    mpc_main_.thrustToSpeed(motors_thrust_, motors_speed_);
    mut_control_.unlock();

    msg_actuators_.header.stamp = ros::Time::now();
    msg_actuators_.angular_velocities[0] = motors_speed_(0);
    msg_actuators_.angular_velocities[1] = motors_speed_(1);
    msg_actuators_.angular_velocities[2] = motors_speed_(2);
    msg_actuators_.angular_velocities[3] = motors_speed_(3);
    pub_motor_command_.publish(msg_actuators_);
  }
}

void MpcRunner::callbackMpcSolve(const ros::TimerEvent &) {
  if (controller_started_) {
    mut_state_.lock();
    ros::Duration elapsed_time = ros::Time::now() - state_time_;
    if (elapsed_time > ros::Duration(0.01)) {
      ROS_WARN("MPC Solve: Outdated state");
    }
    // Here we should account for the solving delay
    mpc_main_.setCurrentState(state_);
    state_ref_ = state_;
    mut_state_.unlock();

    mpc_main_.runMpcStep();

    mut_gains_.lock();
    control_time_ = ros::Time::now();
    motors_thrust_new_ = mpc_main_.getMotorsThrust();
    fb_gains_new_ = mpc_main_.getFeedBackGains();
    flag_new_gains_ = true;
    mut_gains_.unlock();
  }
}

void MpcRunner::callbackConfig(multicopter_mpc_controller::ParamsConfig &config, uint32_t level) {
  ROS_INFO("RECONFIGURE_REQUEST!");
  controller_started_ = config.start_mission;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, ros::this_node::getName());

  MpcRunner mpc_runner;

  ros::AsyncSpinner s(0);
  s.start();
  ros::waitForShutdown();
}