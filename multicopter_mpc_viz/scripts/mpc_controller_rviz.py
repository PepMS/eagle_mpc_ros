#!/usr/bin/env python3

import rospy
import tf

import dynamic_reconfigure.client

from multicopter_mpc_msgs import WholeBodyStatePublisher
from multicopter_mpc_msgs import WholeBodyTrajectoryPublisher

import numpy as np

from multicopter_mpc.utils import MpcMain
from multicopter_mpc.utils import simulator


class MpcController():
    def __init__(self):
        self.mpc_main = MpcMain("passthrough.yaml", "carrot")

        self.N_traj = self.mpc_main.trajectory.n_knots - 1
        self.xs_ref = self.mpc_main.trajectory.getStateTrajectory(0, self.N_traj)
        self.us_ref = self.mpc_main.trajectory.getControlTrajectory(0, self.N_traj - 1)
        self.simulator = simulator.AerialSimulator(self.mpc_main.dt, self.mpc_main.controller.state_reference[0])

        self.xss = []
        self.uss = []
        for i in range(0, self.N_traj):
            self.mpc_main.state = self.simulator.states[-1]
            self.mpc_main.runMpcStep()
            control = np.copy(self.mpc_main.controller.solver.us[0])
            state = self.simulator.simulateStep(control)
            self.xss.append(self.mpc_main.controller.solver.xs)
            self.uss.append(self.mpc_main.controller.solver.us)

        self.xss.append(self.xss[-1])


class MpcControllerNode():
    def __init__(self):
        rospy.init_node('controller', anonymous=True)

        self.rate = rospy.Rate(100)

        self.mpc_controller = MpcController()
        self.xs = self.mpc_controller.simulator.states
        self.us = self.mpc_controller.simulator.controls
        self.us.append(self.us[-1])

        self.qs, self.vs, self.ts = [], [], []
        for x in self.xs:
            self.qs.append(x[:7])
            self.vs.append(x[7:])
            self.ts.append(0.1)

        self.state_pub = WholeBodyStatePublisher('whole_body_state',
                                                 self.mpc_controller.mpc_main.robot_model,
                                                 self.mpc_controller.mpc_main.mc_params,
                                                 frame_id="world")
        self.trajectory_pub = WholeBodyTrajectoryPublisher('whole_body_trajectory',
                                                           self.mpc_controller.mpc_main.robot_model,
                                                           self.mpc_controller.mpc_main.mc_params,
                                                           self.mpc_controller.mpc_main.mission,
                                                           frame_id="world")
        self.partial_trajectory_pub = WholeBodyTrajectoryPublisher('whole_body_partial_trajectory',
                                                                   self.mpc_controller.mpc_main.robot_model,
                                                                   self.mpc_controller.mpc_main.mc_params,
                                                                   self.mpc_controller.mpc_main.mission,
                                                                   frame_id="world")

        self.dyn_rec_client = dynamic_reconfigure.client.Client("trajectory_generator",
                                                                timeout=10,
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
