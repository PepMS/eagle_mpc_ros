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

import crocoddyl

import multicopter_mpc
from multicopter_mpc.utils import simulator, CarrotMpc


class MpcController():
    def __init__(self, trajectoryPath, mpcPath, bagPath=None):
        dtTrajectory = 10
        trajectory = multicopter_mpc.Trajectory()
        trajectory.autoSetup(trajectoryPath)

        problem = trajectory.createProblem(dtTrajectory, True, "IntegratedActionModelEuler")

        solver = multicopter_mpc.SolverSbFDDP(problem, trajectory.squash)
        solver.setCallbacks([crocoddyl.CallbackVerbose()])
        solver.solve([], [], maxiter=400)

        self.mpcController = CarrotMpc(trajectory, solver.xs, dtTrajectory, mpcPath)
        self.mpcController.createProblem_()
        self.mpcController.updateProblem(0)
        self.mpcController.solver.solve(solver.xs[:self.mpcController.problem.T + 1],
                                        solver.us[:self.mpcController.problem.T])

        self.xs = []  # Real state trajectory
        self.us = []  # Real control trajectory

        self.xss = []
        self.uss = []

        if bagPath == "":
            self.nTraj = len(solver.xs)
            self.simulator = simulator.AerialSimulator(self.mpcController.robot_model,
                                                       self.mpcController.platform_params, self.mpcController.dt,
                                                       solver.xs[0])
            time = 0
            for i in range(0, self.nTraj):
                self.mpcController.problem.x0 = self.simulator.states[-1]
                self.mpcController.updateProblem(time)
                self.mpcController.solver.solve(self.mpcController.solver.xs, self.mpcController.solver.us)
                control = np.copy(self.mpcController.solver.us_squash[0])
                self.simulator.simulateStep(control)
                self.xss.append(self.mpcController.solver.xs)
                self.uss.append(self.mpcController.solver.us_squash)
                time += self.mpcController.dt
            self.xs = self.simulator.states
            self.us = self.simulator.controls
        else:
            print()
            #to do
            # if os.path.exists(bagPath):
            #     self.ros_bag = rosbag.Bag(bagPath)
            # else:
            #     sys.exit("Bag " + bagPath + " does not exists")
            # solver_topic = "/solver_performace"
            # solver_msgs = self.ros_bag.read_messages(topics=solver_topic)
            # for topic, msg, t in solver_msgs:
            #     state = np.zeros(13)
            #     state[0] = msg.state_initial.pose.position.x
            #     state[1] = msg.state_initial.pose.position.y
            #     state[2] = msg.state_initial.pose.position.z
            #     state[3] = msg.state_initial.pose.orientation.x
            #     state[4] = msg.state_initial.pose.orientation.y
            #     state[5] = msg.state_initial.pose.orientation.z
            #     state[6] = msg.state_initial.pose.orientation.w
            #     state[7] = msg.state_initial.motion.linear.x
            #     state[8] = msg.state_initial.motion.linear.y
            #     state[9] = msg.state_initial.motion.linear.z
            #     state[10] = msg.state_initial.motion.angular.x
            #     state[11] = msg.state_initial.motion.angular.y
            #     state[12] = msg.state_initial.motion.angular.z

            #     self.xs.append(state)
            #     self.mpc_main.setCurrentState(state)
            #     self.mpc_main.runMpcStep(0)
            #     control = np.copy(self.mpc_main.mpc_controller.getControls(0))
            #     self.us.append(control)
            #     self.xss.append(self.mpc_main.mpc_controller.solver.xs)
            #     self.uss.append(self.mpc_main.mpc_controller.solver.us)

        self.xss.append(self.xss[-1])


class MpcControllerNode():
    def __init__(self):
        rospy.init_node('controller', anonymous=True)

        self.rate = rospy.Rate(100)

        rospack = rospkg.RosPack()
        self.trajectoryPath = rospy.get_param(
            rospy.get_namespace() + "/trajectory_path",
            rospack.get_path('multicopter_mpc_yaml') + '/trajectories/quad_hover.yaml')
        self.mpcPath = rospy.get_param(rospy.get_namespace() + "/mpc_path",
                                       rospack.get_path('multicopter_mpc_yaml') + '/mpc/mpc.yaml')
        self.bagPath = rospy.get_param(rospy.get_namespace() + "/bag_path", "")

        self.mpcController = MpcController(self.trajectoryPath, self.mpcPath, self.bagPath)

        namespace = rospy.get_namespace()
        with open(self.mpcController.mpcController.robot_model_path, "r") as urdf_file:
            urdf_string = urdf_file.read()
        rospy.set_param(namespace + "robot_description", urdf_string)

        self.xs = self.mpcController.xs
        self.us = self.mpcController.us
        self.us.append(self.us[-1])

        self.qs, self.vs, self.ts = [], [], []
        for x in self.xs:
            self.qs.append(x[:7])
            self.vs.append(x[7:])
            self.ts.append(0.1)

        self.statePub = WholeBodyStatePublisher('whole_body_state',
                                                self.mpcController.mpcController.robot_model,
                                                self.mpcController.mpcController.platform_params,
                                                frame_id="world")
        self.trajectoryPub = WholeBodyTrajectoryPublisher('whole_body_trajectory',
                                                          self.mpcController.mpcController.trajectory,
                                                          frame_id="world")
        self.partialTrajectoryPub = WholeBodyTrajectoryPublisher('whole_body_partial_trajectory',
                                                                 self.mpcController.mpcController.trajectory,
                                                                 frame_id="world")

        self.dynRecClient = dynamic_reconfigure.client.Client(
            "/" + rospy.get_param(rospy.get_namespace() + "/dynamic_reconfigure_client"),
            config_callback=self.callbackTrajectoryIdx)

        self.trajectoryTimer = rospy.Timer(rospy.Duration(2), self.callbackTrajectoryTimer)
        self.stateTimer = rospy.Timer(rospy.Duration(0.1), self.callbackStateTimer)

        self.idxTrj = 0

        self.br = tf.TransformBroadcaster()

    def callbackTrajectoryTimer(self, timer):
        self.trajectoryPub.publish(self.ts, self.qs, self.vs)

    def callbackStateTimer(self, timer):
        x = self.xs[self.idxTrj]
        nq = self.mpcController.mpcController.robot_model.nq
        nRotors = self.mpcController.mpcController.platform_params.n_rotors
        self.statePub.publish(0.123, x[:nq], x[nq:], self.us[self.idxTrj][:nRotors], self.us[self.idxTrj][nRotors:])
        qs, vs, ts = [], [], []
        for x in self.mpcController.xss[self.idxTrj]:
            qs.append(x[:nq])
            vs.append(x[nq:])
            ts.append(0.1)
        self.partialTrajectoryPub.publish(ts[0::2], qs[0::2], vs[0::2])

    def callbackTrajectoryIdx(self, config):
        self.trajectory_percentage = config.trajectory_percentage
        self.idxTrj = int((len(self.xs) - 1) * self.trajectory_percentage / 100)


if __name__ == '__main__':
    mpc_node = MpcControllerNode()

    try:
        while not rospy.is_shutdown():
            mpc_node.rate.sleep()

    except rospy.ROSInterruptException:
        pass
