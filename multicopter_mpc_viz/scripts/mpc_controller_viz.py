#!/usr/bin/env python3
import os.path
import sys

import rospy
import rospkg
import rosbag

import tf

import dynamic_reconfigure.client

from multicopter_mpc_viz import WholeBodyStatePublisher
from multicopter_mpc_viz import WholeBodyTrajectoryPublisher

import numpy as np

import example_robot_data

import multicopter_mpc
from multicopter_mpc.utils import simulator


class MpcController():
    def __init__(self, mission_path, mpc_main_path, bag_path):
        self.uav = example_robot_data.loadIris()
        self.uav_model = self.uav.model

        self.mpc_main = multicopter_mpc.MpcMain(multicopter_mpc.MultiCopterType.Iris, mission_path, mpc_main_path)

        self.xs = []  # Real state trajectory
        self.us = []  # Real control trajectory

        self.xss = []
        self.uss = []

        if bag_path == "":
            self.N_traj = self.mpc_main.mpc_controller.trajectory_generator.n_knots - 1
            self.xs_ref = self.mpc_main.mpc_controller.trajectory_generator.states
            self.us_ref = self.mpc_main.mpc_controller.trajectory_generator.controls
            self.simulator = simulator.AerialSimulator(self.mpc_main.mpc_controller.dt, self.xs_ref[0])
            for i in range(0, self.N_traj):
                self.mpc_main.setCurrentState(self.simulator.states[-1])
                self.mpc_main.runMpcStep(0)
                control = np.copy(self.mpc_main.mpc_controller.controls[0])
                self.simulator.simulateStep(control)
                self.xss.append(self.mpc_main.mpc_controller.solver.xs)
                self.uss.append(self.mpc_main.mpc_controller.solver.us)
            self.xs = self.simulator.states
            self.us = self.simulator.controls
        else:
            if os.path.exists(bag_path):
                self.ros_bag = rosbag.Bag(bag_path)
            else:
                sys.exit("Bag " + bag_path + " does not exists")
            solver_topic = "/solver_performace"
            solver_msgs = self.ros_bag.read_messages(topics=solver_topic)
            for topic, msg, t in solver_msgs:
                state = np.zeros(13)
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

                self.xs.append(state)
                self.mpc_main.setCurrentState(state)
                self.mpc_main.runMpcStep(0)
                control = np.copy(self.mpc_main.mpc_controller.getControls(0))
                self.us.append(control)
                self.xss.append(self.mpc_main.mpc_controller.solver.xs)
                self.uss.append(self.mpc_main.mpc_controller.solver.us)

        self.xss.append(self.xss[-1])


class MpcControllerNode():
    def __init__(self):
        rospy.init_node('controller', anonymous=True)

        self.rate = rospy.Rate(100)

        rospack = rospkg.RosPack()
        self.mission_path = rospy.get_param(rospy.get_namespace() + "/mission_path",
                                            rospack.get_path('multicopter_mpc_yaml') + '/missions/takeoff.yaml')
        self.mpc_main_yaml_path = rospy.get_param(rospy.get_namespace() + "/mpc_main_yaml_path",
                                                  rospack.get_path('multicopter_mpc_yaml') + '/mpc_main/mpc-main.yaml')
        self.bag_path = rospy.get_param(rospy.get_namespace() + "/bag_path", "")
        self.mpc_controller = MpcController(self.mission_path, self.mpc_main_yaml_path, self.bag_path)

        self.xs = self.mpc_controller.xs
        self.us = self.mpc_controller.us
        self.us.append(self.us[-1])

        self.qs, self.vs, self.ts = [], [], []
        for x in self.xs:
            self.qs.append(x[:7])
            self.vs.append(x[7:])
            self.ts.append(0.1)

        self.state_pub = WholeBodyStatePublisher('whole_body_state',
                                                 self.mpc_controller.uav_model,
                                                 self.mpc_controller.mpc_main.mpc_controller.mc_params,
                                                 frame_id="world")
        self.trajectory_pub = WholeBodyTrajectoryPublisher(
            'whole_body_trajectory',
            self.mpc_controller.uav_model,
            self.mpc_controller.mpc_main.mpc_controller.mc_params,
            self.mpc_controller.mpc_main.mpc_controller.trajectory_generator.mission,
            frame_id="world")
        self.partial_trajectory_pub = WholeBodyTrajectoryPublisher(
            'whole_body_partial_trajectory',
            self.mpc_controller.uav_model,
            self.mpc_controller.mpc_main.mpc_controller.mc_params,
            self.mpc_controller.mpc_main.mpc_controller.trajectory_generator.mission,
            frame_id="world")

        self.dyn_rec_client = dynamic_reconfigure.client.Client(
            "/" + rospy.get_param(rospy.get_namespace() + "/dynamic_reconfigure_client"),
            config_callback=self.callbackTrajectoryIdx)

        self.trajectory_timer = rospy.Timer(rospy.Duration(2), self.callbackTrajectoryTimer)
        self.state_timer = rospy.Timer(rospy.Duration(0.1), self.callbackStateTimer)

        self.trj_idx = 0

        self.br = tf.TransformBroadcaster()

        self.rotor_names = ['iris__rotor_0', 'iris__rotor_1', 'iris__rotor_2', 'iris__rotor_3']

    def callbackTrajectoryTimer(self, timer):
        self.trajectory_pub.publish(self.ts, self.qs, self.vs)

    def callbackStateTimer(self, timer):
        x = self.xs[self.trj_idx]
        self.state_pub.publish(0.123, x[:7], x[7:], self.us[self.trj_idx], self.rotor_names)
        qs, vs, ts = [], [], []
        for x in self.mpc_controller.xss[self.trj_idx]:
            qs.append(x[:7])
            vs.append(x[7:])
            ts.append(0.1)
        self.partial_trajectory_pub.publish(ts[0::2], qs[0::2], vs[0::2])

    def callbackTrajectoryIdx(self, config):
        self.trajectory_percentage = config.trajectory_percentage
        self.trj_idx = int((len(self.xs) - 1) * self.trajectory_percentage / 100)


if __name__ == '__main__':
    mpc_node = MpcControllerNode()

    try:
        while not rospy.is_shutdown():
            mpc_node.rate.sleep()

    except rospy.ROSInterruptException:
        pass
