from multicopter_mpc_msgs.msg import WholeBodyState, Thrust
import pinocchio
import numpy as np
import copy


class WholeBodyStateInterface():
    def __init__(self, model, mc_params, frame_id="world"):
        print()
        self._model = model
        self._data = self._model.createData()
        self._mc_params = mc_params
        self.frame_id = frame_id
        self._msg = WholeBodyState()
        self._msg.header.frame_id = frame_id

    def writeToMessage(self, t, q, v=None, thrusts=None, rotor_names=None):
        # self._msg.header.stamp = rospy.Time(t)
        self._msg.time = t

        if v is None:
            v = np.zeros(self._model.nv)
        if thrusts is None:
            thrusts = np.zeros(self._mc_params.n_rotors)

        n_rotors = np.size(thrusts)

        pinocchio.centerOfMass(self._model, self._data, q, v)
        c = self._data.com[0]
        cd = self._data.vcom[0]
        # Center of mass
        self._msg.floating_base.pose.position.x = c[0]
        self._msg.floating_base.pose.position.y = c[1]
        self._msg.floating_base.pose.position.z = c[2]
        self._msg.floating_base.motion.linear.x = cd[0]
        self._msg.floating_base.motion.linear.y = cd[1]
        self._msg.floating_base.motion.linear.z = cd[2]
        # Base
        self._msg.floating_base.pose.orientation.x = q[3]
        self._msg.floating_base.pose.orientation.y = q[4]
        self._msg.floating_base.pose.orientation.z = q[5]
        self._msg.floating_base.pose.orientation.w = q[6]
        self._msg.floating_base.motion.angular.x = v[3]
        self._msg.floating_base.motion.angular.y = v[4]
        self._msg.floating_base.motion.angular.z = v[5]
        # Thrusts
        if rotor_names is not None:
            self._msg.thrusts = []
            pinocchio.forwardKinematics(self._model, self._data, q, v)
            for i in range(n_rotors):
                frame_id = self._model.getFrameId(rotor_names[i])
                oMf = pinocchio.updateFramePlacement(self._model, self._data, frame_id)
                pose = pinocchio.SE3ToXYZQUAT(oMf)
                th = Thrust()
                th.pose.position.x = pose[0]
                th.pose.position.y = pose[1]
                th.pose.position.z = pose[2]
                th.pose.orientation.x = pose[3]
                th.pose.orientation.y = pose[4]
                th.pose.orientation.z = pose[5]
                th.pose.orientation.w = pose[6]
                th.thrust_command = thrusts[i]
                th.thrust_min = self._mc_params.min_thrust
                th.thrust_max = self._mc_params.max_thrust
                self._msg.thrusts.append(th)

        return copy.deepcopy(self._msg)