import os.path
import sys
import numpy as np
import rosbag

import multicopter_mpc


class SolverPerformanceIndicator:
    def __init__(self):
        self.iters = 0
        self.final_cost = 0.
        self.solving_time = 0.
        self.state_initial = np.zeros(13)


class MulticopterBag:
    def __init__(self, bag_name):
        if os.path.exists(bag_name):
            self.ros_bag = rosbag.Bag(bag_name)
        else:
            sys.exit("Bag " + bag_name + " does not exists")

        self.solver_iters = []
        self.states = []
        self.controls = []

    def fillSolverPerformanceIndicators(self):
        solver_topic = "/mpc_controller/solver_performance"
        solver_msgs = self.ros_bag.read_messages(topics=solver_topic)
        for topic, msg, t in solver_msgs:
            state = np.zeros(13)
            solver_iter = SolverPerformanceIndicator()
            state[0] = msg.state_initial.pose.position.x
            state[1] = msg.state_initial.pose.position.y
            state[2] = msg.state_initial.pose.position.z
            state[3] = msg.state_initial.pose.orientation.x
            state[4] = msg.state_initial.pose.orientation.y
            state[5] = msg.state_initial.pose.orientation.z
            state[6] = msg.state_initial.pose.orientation.w
            state[7] = msg.state_initial.motion.linear.x
            state[8] = msg.state_initial.motion.linear.y
            state[9] = msg.state_initial.motion.linear.z
            state[10] = msg.state_initial.motion.angular.x
            state[11] = msg.state_initial.motion.angular.y
            state[12] = msg.state_initial.motion.angular.z

            solver_iter.state_initial = state
            solver_iter.solving_time = msg.solving_time
            solver_iter.iters = msg.iters
            solver_iter.final_cost = msg.final_cost

            self.solver_iters.append(solver_iter)

    def fillBagWholeBodyStateTrajectory(self):
        topic_name = "/mpc_controller/solver_performance"
        state_msgs = self.ros_bag.read_messages(topics=topic_name)

        for topic, msg, t in state_msgs:
            state = np.zeros(13)
            state[0] = msg.centroidal.com_position.x
            state[1] = msg.centroidal.com_position.y
            state[2] = msg.centroidal.com_position.z
            state[3] = msg.centroidal.base_orientation.x
            state[4] = msg.centroidal.base_orientation.y
            state[5] = msg.centroidal.base_orientation.z
            state[6] = msg.centroidal.base_orientation.w
            state[7] = msg.centroidal.com_velocity.x
            state[8] = msg.centroidal.com_velocity.y
            state[9] = msg.centroidal.com_velocity.z
            state[10] = msg.centroidal.base_angular_velocity.x
            state[11] = msg.centroidal.base_angular_velocity.y
            state[12] = msg.centroidal.base_angular_velocity.z

            control = np.zeros(4)
            control[0] = msg.thrusts[0].speed_command**2 * self.mc_params.cf
            control[1] = msg.thrusts[1].speed_command**2 * self.mc_params.cf
            control[2] = msg.thrusts[2].speed_command**2 * self.mc_params.cf
            control[3] = msg.thrusts[3].speed_command**2 * self.mc_params.cf

            self.states.append(state)
            self.control.append(control)

    def problemReconstruction(self, mission_path, mpc_main_path):
        self.mpc_main = multicopter_mpc.MpcMain(multicopter_mpc.MultiCopterType.Iris, mission_path, mpc_main_path)

        for idx_iter, s_iter in enumerate(self.solver_iters):
            self.mpc_main.setCurrentState(s_iter.state_initial)
            self.mpc_main.runMpcStep(1)
            control = np.copy(self.mpc_main.mpc_controller.getControls(0))
            print(control)
            print(self.controls[idx_iter])

    #     # self.control_trajectory.pop()

    # def fillDesiredTrajectories(self):
    #     self.trajectory_generator.setSolverCallbacks(True)
    #     self.trajectory_generator.solve()

    # def fillMotorSpeedControl(self):
    #     topic_name = '/mpc_runner_node/motors_state'
    #     motor_msgs = self.ros_bag.read_messages(topics=topic_name)

    #     self.motor_speed_real = []
    #     self.motor_speed_command = []
    #     for topic, msg, t in motor_msgs:
    #         speed_reading = np.array([
    #             msg.thrusts[0].speed_reading, msg.thrusts[1].speed_reading, msg.thrusts[2].speed_reading,
    #             msg.thrusts[3].speed_reading
    #         ])
    #         speed_command = np.array([
    #             msg.thrusts[0].speed_command, msg.thrusts[1].speed_command, msg.thrusts[2].speed_command,
    #             msg.thrusts[3].speed_command
    #         ])
    #         self.motor_speed_real.append(np.copy(speed_reading))
    #         self.motor_speed_command.append(np.copy(speed_command))

    # def fillBagTrajectories(self):
    #     self.fillBagStateTrajectory()
    #     self.fillBagMotorThrustTrajectory()
    #     self.cropBagStateTrajectory()

    # def fillBagStateTrajectory(self):
    #     topic_name = '/iris/ground_truth/odometry'
    #     odometry_msgs = self.ros_bag.read_messages(topics=topic_name)

    #     for topic, msg, t in odometry_msgs:
    #         odom = pinocchio.utils.zero([13, 1])

    #         odom[0] = msg.pose.pose.position.x
    #         odom[1] = msg.pose.pose.position.y
    #         odom[2] = msg.pose.pose.position.z
    #         odom[3] = msg.pose.pose.orientation.x
    #         odom[4] = msg.pose.pose.orientation.y
    #         odom[5] = msg.pose.pose.orientation.z
    #         odom[6] = msg.pose.pose.orientation.w
    #         odom[7] = msg.twist.twist.linear.x
    #         odom[8] = msg.twist.twist.linear.y
    #         odom[9] = msg.twist.twist.linear.z
    #         odom[10] = msg.twist.twist.angular.x
    #         odom[11] = msg.twist.twist.angular.y
    #         odom[12] = msg.twist.twist.angular.z

    #         if hasattr(self, 'bag_state_trajectory'):
    #             self.bag_state_trajectory = np.hstack((self.bag_state_trajectory, odom))
    #             # self.bag_state_trajectory_t = np.hstack(
    #             #     (self.bag_state_trajectory_t,
    #             #      t.nsecs / 1e9 - self.bag_state_trajectory_t0))
    #             self.bag_state_trajectory_t = np.hstack(
    #                 (self.bag_state_trajectory_t, self.bag_state_trajectory_t[-1] + self.dt))
    #         else:
    #             # self.bag_state_trajectory_t0 = t.nsecs / 1e9
    #             # self.bag_state_trajectory_t = t.nsecs / 1e9 - self.bag_state_trajectory_t0
    #             self.bag_state_trajectory_t = np.array([0.0])
    #             self.bag_state_trajectory = np.copy(odom)

    # def fillBagMotorThrustTrajectory(self):
    #     topic_name = '/iris/command/motor_speed'
    #     speed_msgs = self.ros_bag.read_messages(topics=topic_name)

    #     for topic, msg, t in speed_msgs:
    #         speed = np.reshape(np.array(list(msg.angular_velocities)), [4, 1])

    #         if hasattr(self, 'bag_motor_speed'):
    #             self.bag_motor_speed = np.hstack((self.bag_motor_speed, speed))
    #             self.bag_motor_speed_t = np.hstack((self.bag_motor_speed_t, self.bag_motor_speed_t[-1] + self.dt))
    #         else:
    #             self.bag_motor_speed_t = np.array([0.0])
    #             self.bag_motor_speed = np.copy(speed)

    # def cropBagStateTrajectory(self):
    #     self.bag_state_trajectory = self.bag_state_trajectory[:, -len(self.bag_motor_speed[0, :]):]

    # def plotTrajectories(self):
    #     PlotStates([self.desired_state_trajectory, self.bag_state_trajectory], 4e-3)
    #     PlotMotorSpeed([self.desired_motor_speed_trajectory, self.bag_motor_speed], 4e-3)
    #     showPlots()
