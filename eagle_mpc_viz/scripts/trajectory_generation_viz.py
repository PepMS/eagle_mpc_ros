#!/usr/bin/env python3

############################################################################## 
# BSD 3-Clause License
# Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
# All rights reserved. 
##############################################################################

import rospy
import rospkg
import tf

from dynamic_reconfigure.server import Server
from eagle_mpc_viz.cfg import VisualizationConfig

from eagle_mpc_viz import WholeBodyStatePublisher
from eagle_mpc_viz import WholeBodyTrajectoryPublisher

import crocoddyl
import eagle_mpc


class Trajectory():
    def __init__(self, trajectory_path):
        self.trajectory = eagle_mpc.Trajectory()
        self.trajectory.autoSetup(trajectory_path)

    def compute(self, solverType, dt):
        squash = False
        if solverType == 'SolverSbFDDP':
            squash = True

        self.problem = self.trajectory.createProblem(dt, squash, "IntegratedActionModelEuler")

        if solverType == 'SolverSbFDDP':
            self.solver = eagle_mpc.SolverSbFDDP(self.problem, self.trajectory.squash)
        elif solverType == 'SolverBoxFDDP':
            self.solver = crocoddyl.SolverBoxFDDP(self.problem)

        self.solver.setCallbacks([crocoddyl.CallbackVerbose()])
        self.solver.solve([], [], 100)

        if squash:
            us = self.solver.us_squash
        else:
            us = self.solver.us
        return self.solver.xs, us


class TrajectoryNode():
    def __init__(self):
        rospy.init_node('trajectory_generator', anonymous=True)

        self.rate = rospy.Rate(100)

        self.trajectory_path = eagle_mpc.getYamlPath(rospy.get_param(rospy.get_namespace() + "/trajectory_path"))
        self.trajectory = Trajectory(self.trajectory_path)

        self.dt = rospy.get_param(rospy.get_namespace() + "/trajectory_dt", 10)
        self.solverType = rospy.get_param(rospy.get_namespace() + "/trajectory_solver", "SolverSbFDDP")

        namespace = rospy.get_namespace()
        with open(self.trajectory.trajectory.robot_model_path, "r") as urdf_file:
            urdf_string = urdf_file.read()

        rospy.set_param(namespace + "robot_description", urdf_string)

        self.xs, self.us = self.trajectory.compute(self.solverType, self.dt)
        self.us.append(self.us[-1])

        self.continuous_player = False
        self.trajectory_percentage = 0.
        self.trj_idx = 0

        self.srv = Server(VisualizationConfig, self.callbackContinuousReproduction)

        nq = self.trajectory.trajectory.robot_model.nq
        self.qs, self.vs, self.ts = [], [], []
        for x in self.xs:
            self.qs.append(x[:nq])
            self.vs.append(x[nq:])
            self.ts.append(0.1)

        self.state_publisher = WholeBodyStatePublisher('whole_body_state',
                                                       self.trajectory.trajectory.robot_model,
                                                       self.trajectory.trajectory.platform_params,
                                                       frame_id="world")
        self.trajectory_publisher = WholeBodyTrajectoryPublisher('whole_body_trajectory',
                                                                 self.trajectory.trajectory.robot_model,
                                                                 self.trajectory.trajectory.platform_params,
                                                                 self.trajectory.trajectory,
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
            t = self.dt * (self.trj_idx) / 1000
            rospy.loginfo("Trajectory index: %d / %d, Trajectory time: %f", self.trj_idx, len(self.xs) - 1, t)

        return config

    def publishFixedTransform(self):
        self.br.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "world", "map")

    def callbackTrajectoryTimer(self, timer):
        self.trajectory_publisher.publish(self.ts, self.qs, self.vs)
        # print()

    def callbackStateTimer(self, timer):
        x = self.xs[self.trj_idx]
        t = self.dt * (self.trj_idx) / 1000
        if self.continuous_player:
            if self.trj_idx < len(self.xs) - 1:
                self.trj_idx += 1
            else:
                self.trj_idx = 0

        nq = self.trajectory.trajectory.robot_model.nq
        nrotors = self.trajectory.trajectory.platform_params.n_rotors
        self.state_publisher.publish(t, x[:nq], x[nq:], self.us[self.trj_idx][:nrotors],
                                     self.us[self.trj_idx][nrotors:])
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
