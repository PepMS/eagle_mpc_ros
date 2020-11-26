import rospy

import pinocchio

from multicopter_mpc_msgs.msg import WholeBodyTrajectory, Mission, Waypoint
from .whole_body_interface import WholeBodyStateInterface


class WholeBodyTrajectoryPublisher():
    def __init__(self, topic, model, mc_params, mission, frame_id="world", queue_size=10):
        # Defining the subscriber
        self._pub = rospy.Publisher(topic, WholeBodyTrajectory, queue_size=queue_size)
        self._wb_iface = WholeBodyStateInterface(model, mc_params, frame_id)
        self._mission = mission
        self.writeMissionMessage()

    def writeMissionMessage(self):
        self._mission_msg = Mission()
        self._mission_msg.waypoints = []

        for wp in self._mission.waypoints:
            waypoint = Waypoint()
            waypoint.pose.position.x = wp.pose.translation[0]
            waypoint.pose.position.y = wp.pose.translation[1]
            waypoint.pose.position.z = wp.pose.translation[2]
            
            quat = pinocchio.Quaternion(wp.pose.rotation)
            waypoint.pose.orientation.w = quat.w
            waypoint.pose.orientation.x = quat.x
            waypoint.pose.orientation.y = quat.y
            waypoint.pose.orientation.z = quat.z

            waypoint.motion.linear.x = wp.velocity.linear[0]
            waypoint.motion.linear.y = wp.velocity.linear[1]
            waypoint.motion.linear.z = wp.velocity.linear[2]

            waypoint.motion.angular.x = wp.velocity.angular[0]
            waypoint.motion.angular.y = wp.velocity.angular[1]
            waypoint.motion.angular.z = wp.velocity.angular[2]
            self._mission_msg.waypoints.append(waypoint)

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
        msg.mission = self._mission_msg
        
        for i in range(len(ts)):
            vi = None
            if vs is not None:
                vi = vs[i]
            msg.trajectory.append(self._wb_iface.writeToMessage(ts[i], qs[i], vi))
        self._pub.publish(msg)