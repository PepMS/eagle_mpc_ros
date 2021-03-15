import rospy
import dynamic_reconfigure.client
from std_msgs.msg import Bool


class DisturbanceTriggerNode():
    def __init__(self, rate):
        rospy.init_node('disturbance_trigger')

        self.rate = rospy.Rate(rate)

        self.dynRecClient = dynamic_reconfigure.client.Client("/" + rospy.get_namespace() + "/mpc_controller",
                                                              config_callback=self.callbackStartController)

        self.timerDisturbanceTrigger = rospy.Timer(rospy.Duration(0.1), self.callbackDisturbanceTrigger)
        self.disturbanceTriggerPub = rospy.Publisher("/disturbance_enable", Bool, queue_size=1)
        self.controllerStartTime = rospy.Time.now()

    def callbackStartController(self, config):
        self.controllerStarted = config.start_mission
        if self.controllerStarted:
            self.controllerStartTime = rospy.Time.now()

    def callbackDisturbanceTrigger(self, event):
        msg = Bool()
        msg.data = False
        disturbanceStartTime = 2.0
        disturbanceDuration = 2.0
        if (rospy.Time.now() - self.controllerStartTime).to_sec() > disturbanceStartTime and (rospy.Time.now(
        ) - self.controllerStartTime).to_sec() < disturbanceDuration + disturbanceStartTime and self.controllerStarted:
            msg.data = True

        self.disturbanceTriggerPub.publish(msg)


if __name__ == '__main__':
    node = DisturbanceTriggerNode(10)

    try:
        while not rospy.is_shutdown():
            node.rate.sleep()

    except rospy.ROSInterruptException:
        pass
