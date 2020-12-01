#!/usr/bin/env python3

import rospy
import rospkg

import tf

from dynamic_reconfigure.server import Server
from multicopter_mpc_viz.cfg import VisualizationConfig

from multicopter_mpc_viz import WholeBodyStatePublisher
from multicopter_mpc_viz import WholeBodyTrajectoryPublisher

import pinocchio
import example_robot_data

import multicopter_mpc
from multicopter_mpc.utils.path import MULTICOPTER_MPC_MULTIROTOR_DIR


class Trajectory():
    def __init__(self, mission_path, trajectory_generation_path):
        self.uav = example_robot_data.loadIris()
        self.uav_model = self.uav.model

        # # UAV Params
        self.mc_params = multicopter_mpc.MultiCopterBaseParams()
        self.mc_params.fill(MULTICOPTER_MPC_MULTIROTOR_DIR + "/iris.yaml")

        # Mission
        self.mission = multicopter_mpc.Mission(self.uav.nq + self.uav.nv)
        self.mission.fillWaypoints(mission_path)

        self.trajectory = multicopter_mpc.TrajectoryGenerator(self.uav_model, self.mc_params, self.mission)
        self.trajectory.loadParameters(trajectory_generation_path)
        self.dt = self.trajectory.dt
        self.trajectory.createProblem(multicopter_mpc.SolverType.SolverTypeSquashBoxFDDP,
                                      multicopter_mpc.IntegratorType.IntegratorTypeEuler, self.dt)
        self.trajectory.setSolverCallbacks(True)

    def compute(self):
        self.trajectory.setSolverIters(100)
        self.trajectory.setSolverCallbacks(True)
        self.trajectory.solve()

        return self.trajectory.states, self.trajectory.controls


class TrajectoryNode():
    def __init__(self):
        rospy.init_node('trajectory_generator', anonymous=True)

        self.rate = rospy.Rate(100)

        rospack = rospkg.RosPack()
        self.mission_path = rospy.get_param(rospy.get_namespace() + "/mission_path",
                                            rospack.get_path('multicopter_mpc_yaml') + '/missions/takeoff.yaml')
        self.trajectory_generation_yaml_path = rospy.get_param(
            rospy.get_namespace() + "/trajectory_generation_yaml_path",
            rospack.get_path('multicopter_mpc_yaml') + '/trajectory_generation/trajectory-generator.yaml')

        self.trajectory = Trajectory(self.mission_path, self.trajectory_generation_yaml_path)
        self.xs, self.us = self.trajectory.compute()
        self.us.append(self.us[-1])

        self.rotor_names = ['iris__rotor_0', 'iris__rotor_1', 'iris__rotor_2', 'iris__rotor_3']
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
                                                       self.trajectory.uav_model,
                                                       self.trajectory.mc_params,
                                                       frame_id="world")
        self.trajectory_publisher = WholeBodyTrajectoryPublisher('whole_body_trajectory',
                                                                 self.trajectory.uav_model,
                                                                 self.trajectory.mc_params,
                                                                 self.trajectory.mission,
                                                                 frame_id="world")

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
        self.trajectory_publisher.publish(self.ts, self.qs, self.vs)

    def callbackStateTimer(self, timer):
        x = self.xs[self.trj_idx]
        if self.continuous_player:
            if self.trj_idx < len(self.xs) - 1:
                self.trj_idx += 1
            else:
                self.trj_idx = 0

        self.state_publisher.publish(0.123, x[:7], x[7:], self.us[self.trj_idx], self.rotor_names)
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
