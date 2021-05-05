import os.path
import sys
import re
import numpy as np
import bisect

import rosbag

import multicopter_mpc


class SolverPerformanceIndicator:
    def __init__(self):
        self.iters = 0
        self.final_cost = 0.
        self.solving_time = 0.
        self.state_initial = np.zeros(13)
        self.time = 0.


class MulticopterBag:
    def __init__(self, bag_path, trajectory_path):
        if os.path.exists(bag_path):
            self.ros_bag = rosbag.Bag(bag_path)
        else:
            sys.exit("Bag file " + bag_path + " does not exists")

        self.trajectory = multicopter_mpc.Trajectory()
        self.trajectory.autoSetup(trajectory_path)
        self.robot_model = self.trajectory.robot_model

        m = re.search("(.+)__base_link", self.trajectory.platform_params.base_link_name)
        self.namespace = "/" + m.group(1) + "/"

        self.states = []
        self.states_t = []
        self.states_platform = []
        self.states_platform_t = []
        self.states_joints = []
        self.states_joints_t = []

        self.controls = []
        self.controls_t = []
        self.control_rotors = []
        self.control_rotors_t = []
        self.control_joints = []
        self.control_joints_t = []

        self.disturbances = []
        self.disturbances_t = []

        self.solving_time = []
        self.solving_cost = []
        self.solving_iters = []
        self.solving_time_t = []

        self.fill_controls()
        self.fill_states()

    def fill_states(self):
        topic_name = self.namespace + "ground_truth/odometry"
        state_msgs = self.ros_bag.read_messages(topics=topic_name)

        for topic, msg, t in state_msgs:
            state = np.zeros(13)
            state[0] = msg.pose.pose.position.x
            state[1] = msg.pose.pose.position.y
            state[2] = msg.pose.pose.position.z
            state[3] = msg.pose.pose.orientation.x
            state[4] = msg.pose.pose.orientation.y
            state[5] = msg.pose.pose.orientation.z
            state[6] = msg.pose.pose.orientation.w
            state[7] = msg.twist.twist.linear.x
            state[8] = msg.twist.twist.linear.y
            state[9] = msg.twist.twist.linear.z
            state[10] = msg.twist.twist.angular.x
            state[11] = msg.twist.twist.angular.y
            state[12] = msg.twist.twist.angular.z

            self.states_platform.append(state)
            self.states_platform_t.append(msg.header.stamp.to_sec())

        topic_name = self.namespace + "joint_states"
        state_msgs = self.ros_bag.read_messages(topics=topic_name)

        nj = self.robot_model.nq - 7
        for topic, msg, t in state_msgs:
            if msg.name[0].startswith("flying_arm"):
                state = np.zeros(nj * 2)
                for i in range(nj):
                    state[i] = msg.position[i]
                    state[i + nj] = msg.velocity[i]
                self.states_joints.append(state)
                self.states_joints_t.append(msg.header.stamp.to_sec())

        for s, s_t in zip(self.states_platform, self.states_platform_t):
            if s_t >= self.t_ini:
                nq = self.trajectory.state.nq
                nj = self.trajectory.state.nq - 7
                nx = self.trajectory.state.nx
                state = np.zeros(nx)
                state[:7] = s[:7]
                state[nq:nq + 6] = s[7:]
                s_j_idx = bisect.bisect_right(self.states_joints_t, s_t)
                s_j = self.states_joints[s_j_idx - 1]
                state[7:7 + nj] = s_j[:nj]
                state[-nj:] = s_j[-nj:]
                self.states.append(state)
                self.states_t.append(s_t - self.t_ini)

    def fill_controls(self):
        topic_name = self.namespace + "motor_speed"
        state_msgs = self.ros_bag.read_messages(topics=topic_name)

        for topic, msg, t in state_msgs:
            nr = self.trajectory.platform_params.n_rotors
            control = np.zeros(nr)
            for i in range(nr):
                control[i] = msg.angular_velocities[i]**2 * self.trajectory.platform_params.cf
            self.control_rotors.append(control)
            self.control_rotors_t.append(msg.header.stamp.to_sec())

        topic_name = self.namespace + "joint_states"
        state_msgs = self.ros_bag.read_messages(topics=topic_name)

        nj = self.robot_model.nq - 7
        for topic, msg, t in state_msgs:
            if msg.name[0].startswith("flying_arm"):
                control = np.zeros(nj)
                for i in range(nj):
                    control[i] = msg.effort[i]
                self.control_joints.append(control)
                self.control_joints_t.append(msg.header.stamp.to_sec())

        for idx, control in enumerate(self.control_rotors):
            c = np.linalg.norm(control)
            if c > 1e-1:
                self.t_ini = self.control_rotors_t[idx]
                break

        for c, c_t in zip(self.control_rotors, self.control_rotors_t):
            if c_t >= self.t_ini:
                nu = self.trajectory.actuation.nu
                nr = self.trajectory.platform_params.n_rotors
                control = np.zeros(nu)
                control[:nr] = c[:nr]
                c_j_idx = bisect.bisect_right(self.control_joints_t, c_t)
                c_j = self.control_joints[c_j_idx - 1]
                control[nr:] = c_j[:]
                self.controls.append(control)
                self.controls_t.append(c_t - self.t_ini)

    def fill_disturbances(self):
        topic_name = self.namespace + "external_force"
        dist_msgs = self.ros_bag.read_messages(topics=topic_name)

        d_norm_last = 0
        self.dist_magnitude = 0
        self.dist_t_ini = 0
        self.dist_t_end = 0
        self.dist_duration = 0
        for topic, msg, t in dist_msgs:
            time = msg.header.stamp.to_sec()
            if time >= self.t_ini:
                dist = np.zeros(3)
                dist[0] = msg.wrench.force.x
                dist[1] = msg.wrench.force.y
                dist[2] = msg.wrench.force.z

                d_norm = np.linalg.norm(dist)
                if d_norm_last < 1e-1 and d_norm > 1e-1:
                    self.dist_magnitude = d_norm
                    self.dist_t_ini = time - self.t_ini

                if d_norm_last > 1e-1 and d_norm < 1e-1:
                    self.dist_t_end = time - self.t_ini
                    self.dist_duration = self.dist_t_end - self.dist_t_ini
                self.disturbances.append(dist)
                self.disturbances_t.append(time - self.t_ini)

                d_norm_last = d_norm
    
    def fill_solver(self):
        topic_name = "/solver_performance"
        solver_msgs = self.ros_bag.read_messages(topics=topic_name)

        for topic, msg, t in solver_msgs:
            time = msg.header.stamp.to_sec()
            self.solving_time.append(msg.solving_time.to_sec())
            self.solving_cost.append(msg.final_cost)
            self.solving_iters.append(msg.iters + 1)
            self.solving_time_t.append(time)

    # def problemReconstruction(self, mpc_main_path):
    #     self.mpc_main = multicopter_mpc.MpcMain(multicopter_mpc.MultiCopterType.Iris, self.mission_path, mpc_main_path)

    #     for idx_iter, s_iter in enumerate(self.solver_iters):
    #         self.mpc_main.setCurrentState(s_iter.state_initial)
    #         self.mpc_main.runMpcStep(1)
    #         control = np.copy(self.mpc_main.mpc_controller.getControls(0))
    #         print(control)
    #         print(self.controls[idx_iter])

    # def fillSolverPerformanceIndicators(self):
    #     solver_topic = "/mpc_controller/solver_performance"
    #     solver_msgs = self.ros_bag.read_messages(topics=solver_topic)
    #     for topic, msg, t in solver_msgs:
    #         state = np.zeros(13)
    #         solver_iter = SolverPerformanceIndicator()
    #         state[0] = msg.state_initial.pose.position.x
    #         state[1] = msg.state_initial.pose.position.y
    #         state[2] = msg.state_initial.pose.position.z
    #         state[3] = msg.state_initial.pose.orientation.x
    #         state[4] = msg.state_initial.pose.orientation.y
    #         state[5] = msg.state_initial.pose.orientation.z
    #         state[6] = msg.state_initial.pose.orientation.w
    #         state[7] = msg.state_initial.motion.linear.x
    #         state[8] = msg.state_initial.motion.linear.y
    #         state[9] = msg.state_initial.motion.linear.z
    #         state[10] = msg.state_initial.motion.angular.x
    #         state[11] = msg.state_initial.motion.angular.y
    #         state[12] = msg.state_initial.motion.angular.z

    #         solver_iter.state_initial = state
    #         solver_iter.solving_time = msg.solving_time.secs + msg.solving_time.nsecs / 1e9
    #         solver_iter.iters = msg.iters
    #         solver_iter.final_cost = msg.final_cost
    #         solver_iter.time = t.secs + t.nsecs / 1e9
    #         self.solver_iters.append(solver_iter)