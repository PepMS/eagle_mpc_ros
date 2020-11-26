#!/usr/bin/env python3

import rospy
import tf

from dynamic_reconfigure.server import Server
from multicopter_mpc_viz.cfg import VisualizationConfig

from multicopter_mpc_viz import WholeBodyStatePublisher
from multicopter_mpc_viz import WholeBodyTrajectoryPublisher

import pinocchio
import example_robot_data

import multicopter_mpc
from multicopter_mpc.utils.path import MULTICOPTER_MPC_MULTIROTOR_DIR, MULTICOPTER_MPC_MISSION_DIR


class Trajectory():
    def __init__(self, mission, use_internal_gains=False):
        self.uav = example_robot_data.loadIris()
        self.uav_model = self.uav.model

        # # UAV Params
        self.mc_params = multicopter_mpc.MultiCopterBaseParams()
        self.mc_params.fill(MULTICOPTER_MPC_MULTIROTOR_DIR + "/iris.yaml")

        # Mission
        self.mission = multicopter_mpc.Mission(self.uav.nq + self.uav.nv)
        self.mission.fillWaypoints(MULTICOPTER_MPC_MISSION_DIR + "/" + mission)

        if use_internal_gains:
            trajectory_yaml_path = "/home/pepms/wsros/mpc-ws/src/multicopter_mpc_ros/yaml/feedback-gains/trajectory-generator.yaml"
        else:
            trajectory_yaml_path = "/home/pepms/wsros/mpc-ws/src/multicopter_mpc/multicopter_mpc_yaml/trajectory_generation/trajectory-generator.yaml"

        self.trajectory = multicopter_mpc.TrajectoryGenerator(self.uav_model, self.mc_params, self.mission)
        self.trajectory.loadParameters(trajectory_yaml_path)
        self.dt = self.trajectory.dt
        self.trajectory.createProblem(multicopter_mpc.SolverType.SolverTypeBoxFDDP,
                                      multicopter_mpc.IntegratorType.IntegratorTypeEuler, self.dt)
        self.trajectory.setSolverCallbacks(True)

    def compute(self):
        state_guess = self.mission.interpolateTrajectory("cold")
        control = pinocchio.utils.zero(4)
        control_guess = [control for _ in range(0, len(state_guess) - 1)]
        self.trajectory.setSolverIters(100)
        self.trajectory.setSolverCallbacks(True)
        self.trajectory.solve(state_guess, control_guess)

        return self.trajectory.solver.xs, self.trajectory.solver.us


class TrajectoryNode():
    def __init__(self):
        rospy.init_node('trajectory_generator', anonymous=True)

        self.rate = rospy.Rate(100)

        self.mission_name = rospy.get_param(rospy.get_name() + "/mission_type", "takeoff.yaml")
        self.feedback_gains = rospy.get_param(rospy.get_name() + "/use_internal_gains", False)

        self.trajectory = Trajectory(self.mission_name)
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
