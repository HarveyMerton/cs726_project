#! /usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan

class SensorLaserscan():

    def __init__(self):
        # Initialise
        rospy.init_node('SensorLaserscanNode')

        # Subscribers
        rospy.Subscriber('/scan', LaserScan, self.callback_scan)

        # Publishers
        self.pub_laserscan = rospy.Publisher('/sens_scan', LaserScan, queue_size=10)
        
        rospy.spin()

    ### DEFINE CALLBACKS ###
    def callback_scan(self, data_laserscan):
        laserscan_gaps = LaserScan()  # To store array of gaps and obstacles

        index_last = 0  # Index of last obstacle seen

        gap_width_req = 0.5  # Required gap width for robot to fit through
        num_points = len(data_laserscan.ranges)  # Number of laser scanner points
        ranges_thresholded = [0.0 for j in range(num_points)]  # Binary array storing 0's
                                                             # if a gap is present, 1's if not

        # Iterate through all points in laser scan FOV
        for i in range(num_points):
            if (data_laserscan.ranges[i] == 'nan'):  # Treat out of range as a gap
                ranges_thresholded[i] = 1
            if (data_laserscan.ranges[i] < 1):  # Obstacle threshold
                ranges_thresholded[i] = 0.0

                # Find gap size
                if i != 0 and i != num_points and ranges_thresholded[i] != ranges_thresholded[i-1]:

                    gap = i - index_last  # Gap range across array
                    a = data_laserscan.ranges[index_last]  # Distances a and b of triangle formed
                    b = data_laserscan.ranges[i]

                    avg = (a+b)/2  # Average distance to gap

                    angle = gap * 0.001637 
                    angle_req = math.acos((2*(avg**2) - gap_width_req**2)/(2*(avg**2)))

                    # Check if gap is too small
                    if angle < angle_req and index_last != 0:
                        for j in range(index_last, i):  # Gap too small, treat as wall
                            ranges_thresholded[j] = 0
                index_last = i
            else:
                ranges_thresholded[i] = 1.0

        # Publish thresholded ranges array
        laserscan_gaps.ranges = ranges_thresholded
        self.pub_laserscan.publish(laserscan_gaps)

if __name__ == '__main__':
    SensorLaserscan()




