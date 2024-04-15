#! /usr/bin/env python

import rospy
from kobuki_msgs.msg import BumperEvent

class SensorBumper():

    def __init__(self):
        # Initialise
        rospy.init_node('SensorBumperNode')
        rospy.on_shutdown(self.shutdown)

        # Subscribers
        rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.RobotBumpedCallback)

        # Publishers
        self.pub_bumper = rospy.Publisher('/sens_bump', BumperEvent, queue_size=10)

        rospy.spin()

    ### DEFINE CALLBACKS ###
    def RobotBumpedCallback(self, data):
        self.pub_bumper.publish(data)

    ### DEFINE SHUTDOWN ###
    def shutdown(self):
        rospy.sleep(1)

if __name__ == '__main__':
    SensorBumper()