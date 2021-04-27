from multicopter_mpc_msgs.msg import WholeBodyState, Thrust, JointState
import pinocchio
import numpy as np
import copy


class WholeBodyStateInterface():
    def __init__(self, model, platform_params, frame_id="world"):
        self._model = model
        self._data = self._model.createData()
        self._platform_params = platform_params
        self.frame_id = frame_id
        self._msg = WholeBodyState()
        self._msg.header.frame_id = frame_id

    def writeToMessage(self, t, q, v=None, thrusts=None, tau=None, rotor_names=None):
        # self._msg.header.stamp = rospy.Time(t)
        self._msg.time = t

        if v is None:
            v = np.zeros(self._model.nv)
        if thrusts is None:
            thrusts = np.zeros(self._platform_params.n_rotors)

        n_rotors = np.size(thrusts)

        self._msg.floating_base.pose.position.x = q[0]
        self._msg.floating_base.pose.position.y = q[1]
        self._msg.floating_base.pose.position.z = q[2]
        self._msg.floating_base.motion.linear.x = v[0]
        self._msg.floating_base.motion.linear.y = v[1]
        self._msg.floating_base.motion.linear.z = v[2]
        # Base
        self._msg.floating_base.pose.orientation.x = q[3]
        self._msg.floating_base.pose.orientation.y = q[4]
        self._msg.floating_base.pose.orientation.z = q[5]
        self._msg.floating_base.pose.orientation.w = q[6]
        self._msg.floating_base.motion.angular.x = v[3]
        self._msg.floating_base.motion.angular.y = v[4]
        self._msg.floating_base.motion.angular.z = v[5]
        # Thrusts
        self._msg.thrusts = []
        pinocchio.forwardKinematics(self._model, self._data, q, v)
        frame_id = self._model.getFrameId(self._platform_params.base_link_name)
        iMbl = pinocchio.updateFramePlacement(self._model, self._data, frame_id)
        for i in range(n_rotors):
            th = Thrust()
            iMrot = iMbl * self._platform_params.rotors_pose[i]
            pose = pinocchio.SE3ToXYZQUAT(iMrot)
            th.pose.position.x = pose[0]
            th.pose.position.y = pose[1]
            th.pose.position.z = pose[2]
            th.pose.orientation.x = pose[3]
            th.pose.orientation.y = pose[4]
            th.pose.orientation.z = pose[5]
            th.pose.orientation.w = pose[6]
            th.thrust_command = thrusts[i]
            th.thrust_min = self._platform_params.min_thrust
            th.thrust_max = self._platform_params.max_thrust
            self._msg.thrusts.append(th)

        self._msg.joints = []
        njoints = self._model.njoints - 2
        for j in range(njoints):
            joint = JointState()
            joint.name = self._model.names[j + 2]
            joint.position = q[7 + j]
            joint.velocity = v[6 + j]
            if tau is not None:
                joint.effort = tau[j]
            self._msg.joints.append(joint)

        return copy.deepcopy(self._msg)