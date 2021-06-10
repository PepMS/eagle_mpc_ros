############################################################################## 
# BSD 3-Clause License
# Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
# All rights reserved. 
##############################################################################

import rospy

from eagle_mpc_msgs.msg import WholeBodyState
from .whole_body_interface import WholeBodyStateInterface


class WholeBodyStatePublisher():
    def __init__(self, topic, model, platform_params, frame_id="world", queue_size=10):
        # Initializing the publisher
        self._pub = rospy.Publisher(topic, WholeBodyState, queue_size=queue_size)
        self._wb_iface = WholeBodyStateInterface(model, platform_params, frame_id)

    def publish(self, t, q, v, thrusts, tau):
        msg = self._wb_iface.writeToMessage(t, q, v, thrusts, tau)
        self._pub.publish(msg)