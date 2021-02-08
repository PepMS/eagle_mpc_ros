#!/usr/bin/env python3

import rospy
import rospkg
import tf

import numpy as np

from dynamic_reconfigure.server import Server
from multicopter_mpc_viz.cfg import VisualizationConfig

from multicopter_mpc_viz import WholeBodyStatePublisher
from multicopter_mpc_viz import WholeBodyTrajectoryPublisher

import pinocchio
import crocoddyl
import example_robot_data

import multicopter_mpc
from multicopter_mpc.utils.path import MULTICOPTER_MPC_MULTIROTOR_DIR


class Trajectory():
    def __init__(self, trajectory_name, trajectory_path):
        self.trajectory = multicopter_mpc.Trajectory()
        self.parserYaml = multicopter_mpc.ParserYaml(trajectory_name, trajectory_path)
        self.paramsServer = multicopter_mpc.ParamsServer(self.parserYaml.params())
        self.trajectory.autoSetup(self.paramsServer)

    def compute(self):
        self.initialState = self.trajectory.state.zero()
        self.problem = self.trajectory.createProblem(10, True, self.initialState, "IntegratedActionModelEuler")
        self.solver = multicopter_mpc.SolverSbFDDP(self.problem, self.trajectory.squash)

        self.solver.setCallbacks([crocoddyl.CallbackVerbose()])
        self.solver.solve([], [], 100)

        return self.solver.xs, self.solver.us_squash


class TrajectoryNode():
    def __init__(self):
        rospy.init_node('trajectory_generator', anonymous=True)

        self.rate = rospy.Rate(100)

        rospack = rospkg.RosPack()

        self.trajectory_name = rospy.get_param(
            rospy.get_namespace() + "/trajectory_name",
            rospack.get_path('multicopter_mpc_yaml') + '/trajectories/quad_hover.yaml')
        self.trajectory_path = rospy.get_param(rospy.get_namespace() + "/trajectory_path",
                                               rospack.get_path('multicopter_mpc_yaml') + '/trajectories')
        self.trajectory = Trajectory(self.trajectory_name, self.trajectory_name)

        self.xs, self.us = self.trajectory.compute()
        self.us.append(self.us[-1])

        # self.rotor_names = ['rotor_0', 'rotor_1', 'rotor_2', 'rotor_3']
        self.continuous_player = False
        self.trajectory_percentage = 0.
        self.trj_idx = 0

        self.srv = Server(VisualizationConfig, self.callbackContinuousReproduction)

        self.qs, self.vs, self.ts = [], [], []
        for x in self.xs:
            self.qs.append(x[:7])
            self.vs.append(x[7:])
            self.ts.append(0.1)

        self.state_publisher = WholeBodyStatePublisher('whole_body_state',
                                                       self.trajectory.trajectory.robot_model,
                                                       self.trajectory.trajectory.platform_params,
                                                       frame_id="world")
        # self.trajectory_publisher = WholeBodyTrajectoryPublisher('whole_body_trajectory',
        #                                                          self.trajectory.trajectory.robot_model,
        #                                                          self.trajectory.trajectory.platform_params,
        #                                                          self.trajectory.mission,
        #                                                          frame_id="world")

        self.trajectory_timer = rospy.Timer(rospy.Duration(2), self.callbackTrajectoryTimer)
        self.state_timer = rospy.Timer(rospy.Duration(0.1), self.callbackStateTimer)

        self.br = tf.TransformBroadcaster()

    def callbackContinuousReproduction(self, config, level):
        self.continuous_player = config.continuous_player

        if self.trajectory_percentage != config.trajectory_percentage:
            self.trajectory_percentage = config.trajectory_percentage
            self.continuous_player = False
            config.continuous_player = False
            self.trj_idx = int((len(self.xs) - 1) * self.trajectory_percentage / 100)
            rospy.loginfo("Trajectory index: %d / %d", self.trj_idx, len(self.xs) - 1)

        return config

    def publishFixedTransform(self):
        self.br.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "world", "map")

    def callbackTrajectoryTimer(self, timer):
        # self.trajectory_publisher.publish(self.ts, self.qs, self.vs)
        print()

    def callbackStateTimer(self, timer):
        x = self.xs[self.trj_idx]
        if self.continuous_player:
            if self.trj_idx < len(self.xs) - 1:
                self.trj_idx += 1
            else:
                self.trj_idx = 0

        self.state_publisher.publish(0.123, x[:7], x[7:], self.us[self.trj_idx])
        self.publishFixedTransform()

        if self.continuous_player:
            self.trajectory_percentage = self.trj_idx / (len(self.xs) - 1) * 100.0
            self.srv.update_configuration({"trajectory_percentage": self.trajectory_percentage})


if __name__ == '__main__':
    trj_node = TrajectoryNode()

    try:
        while not rospy.is_shutdown():
            trj_node.rate.sleep()

    except rospy.ROSInterruptException:
        pass
