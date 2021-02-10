import rospy

import pinocchio
import crocoddyl
import multicopter_mpc

from multicopter_mpc_msgs.msg import WholeBodyTrajectory, Trajectory, Placement
from .whole_body_interface import WholeBodyStateInterface


class WholeBodyTrajectoryPublisher():
    def __init__(self, topic, trajectory, frame_id="world", queue_size=10):
        # Defining the subscriber
        self._pub = rospy.Publisher(topic, WholeBodyTrajectory, queue_size=queue_size)
        self._wb_iface = WholeBodyStateInterface(trajectory.robot_model, trajectory.platform_params, frame_id)
        self._trajectory = trajectory
        self._placements = []
        for stage in self._trajectory.stages:
            for cost in stage.cost_types.todict():
                if stage.cost_types.todict()[cost] == multicopter_mpc.CostModelTypes.CostModelFramePlacement:
                    self._placements.append(stage.costs.costs[cost].cost.reference.placement)

        self.writeTrajectoryMessage()

    def writeTrajectoryMessage(self):
        self._trajectory_msg = Trajectory()
        self._trajectory_msg.placements = []

        for place in self._placements:
            placement = Placement()
            placement.pose.position.x = place.translation[0]
            placement.pose.position.y = place.translation[1]
            placement.pose.position.z = place.translation[2]

            quat = pinocchio.Quaternion(place.rotation)
            placement.pose.orientation.w = quat.w
            placement.pose.orientation.x = quat.x
            placement.pose.orientation.y = quat.y
            placement.pose.orientation.z = quat.z

            self._trajectory_msg.placements.append(placement)

    def publish(self, ts, qs, vs=None):
        msg = WholeBodyTrajectory()
        # Check that the length of the lists are consistent
        if len(ts) != len(qs):
            print("Couldn't publish the message since the length of the qs list is not consistent")
            print("Len ts", len(ts))
            print("Len qs", len(qs))
            return
        if vs is not None:
            if len(ts) != len(vs):
                print("Couldn't publish the message since the length of the vs list is not consistent")
                return

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self._wb_iface.frame_id
        msg.trajectory = self._trajectory_msg

        for i in range(len(ts)):
            vi = None
            if vs is not None:
                vi = vs[i]
            msg.robot_state_trajectory.append(self._wb_iface.writeToMessage(ts[i], qs[i], vi))
        self._pub.publish(msg)