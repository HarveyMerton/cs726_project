#! /usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class SensorImu():
    def __init__(self):
        # Initialise
        rospy.init_node('SensorImuNode', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        # Subscribers
        rospy.Subscriber('/odom', Odometry, self.current_pos)

        # Publishers
        self.pub_imu = rospy.Publisher('/sens_imu', Twist, queue_size=10)

        # Instance variables
        self.currentAngle = 0.0
        self.currentX = 0.0
        self.currentY = 0.0

        rospy.spin()

    ### DEFINE CALLBACKS ###
    def current_pos(self, current_odom):
        # ANGLE
        # Convert angular information into euler co-ordinates
        orientation_q = current_odom.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.currentAngle) = euler_from_quaternion(orientation_list)

        # Convert to degrees 0 to 360 with 0 deg being initial forward direction,
        # counting up CCW
        if self.currentAngle < 0: 
            self.currentAngle = 2*math.pi + self.currentAngle

        # POSITION
        self.currentX = current_odom.pose.pose.position.x
        self.currentY = current_odom.pose.pose.position.y

        # Publish the current position
        current_pose = Twist()
        current_pose.angular.z = self.currentAngle*(180.0/math.pi)  # Convert to degrees
        current_pose.linear.x = self.currentX
        current_pose.linear.y = self.currentY

        self.pub_imu.publish(current_pose)

    ### DEFINE SHUTDOWN ###
    def shutdown(self):
        rospy.sleep(1)
    
if __name__ == '__main__':
    SensorImu()
