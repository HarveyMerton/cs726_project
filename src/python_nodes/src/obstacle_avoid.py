#! /usr/bin/env python

import rospy

from geometry_msgs.msg import Twist

from std_msgs.msg import String
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import LaserScan

class ObstacleAvoidNode():

    def __init__(self):
        # Initialise node
        rospy.init_node('ObstacleAvoidanceNode', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        
        # Subscribers
        rospy.Subscriber('/sens_scan', LaserScan, self.callback_sens_scan)
        rospy.Subscriber('/sens_bump', BumperEvent, self.callback_sens_bump)
        rospy.Subscriber('/sens_imu', Twist, self.callback_sens_imu)
 
        # Publishers
        self.cmd_vel_twist = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.cmd_take_photo = rospy.Publisher('/cmd_photo', String, queue_size=10)

        # Initialise speed variables
        self.speed_base_lin = 1
        self.speed_base_ang = 1

        self.speed_lineup_lin = 0.8
        self.speed_lineup_ang = 1

        self.speed_bumper_lin = 0.4
        self.speed_bumper_ang = 1

        self.speed_lidar_lin = 0.2
        self.speed_lidar_ang = 1

        # Initialise position variables
        self.current_angle = 0.0
        self.current_x = 0.0
        self.current_y = 0.0

        self.hit_pt_x = 0.0
        self.hit_pt_y = 0.0

        # Define state tracking variables
        self.state = 1  # State in the obstacle avoidance finite state machine
        side_selected = 0  # Flag to store if a side of an obstacle to track has been selected

        self.which_dir = 0  # Tracks direction moving around the obstacle 1 = left -1 = right

        self.state_bumper = False  # Flag to track if the robot is reacting to bumper inputs
        self.which_bumper = 0  # Tracks which bumper was pressed

        self.flag_returning = False  # Flag to track when robot returning to centre line & should ignore
                                     # object in LiDAR view
        self.flag_changed_dir = False  # Flag to check if robot has already changed direction during obstacle avoid
        self.flag_passed_hit = False # Flag to track if the robot has passed its initial hit point on an obstacle it
                                    # turns around on

        # Define LiDAR scan_regions
        self.scan_data = LaserScan()
        self.scan_regions = [[1 for i in range(20)], [1 for i in range(191)],
                             [1 for i in range(215)], [1 for i in range(191)],
                             [1 for i in range(20)]]

        # Define rates
        r = rospy.Rate(100)  # Refresh rate
        pic_delay = rospy.Rate(0.9)  # Delay for taking pictures

        d = rospy.Rate(1)  # Delay before starting to give other nodes time to start
        d.sleep()

        # Start FSM
        while not rospy.is_shutdown():
            # Drive forward along centre line
            if self.state == 1:
                side_selected = 0
                self.act_follow_cline()

            # Avoid an obstacle
            elif self.state == 2:
                if side_selected == 0:
                    side_selected = 1

                    # Take a photo of the object if it is newly encountered
                    self.cmd_take_photo.publish('')
                    self.cmd_vel_twist.publish(Twist())  # Briefly stop for photo
                    pic_delay.sleep()

                    # Determine which side of the robot is more clear 1 for left, -1 for right
                    if sum(self.scan_data.ranges[0:319]) > sum(self.scan_data.ranges[320:639]):
                        self.which_dir = -1  # More gaps to right - turn right
                    else:
                        self.which_dir = 1  # More gaps to left - turn left

                self.act_avoid()

            r.sleep()


    ### DEFINE CALLBACKS ###
    # Callback for imu
    def callback_sens_imu(self, data_imu):
        self.current_angle = data_imu.angular.z
        self.current_x = data_imu.linear.x
        self.current_y = data_imu.linear.y

    # Callback for bumper
    def callback_sens_bump(self, data_bump):
        if (data_bump.state == BumperEvent.PRESSED):  # If a bumper is pressed
            self.which_bumper = data_bump.bumper
            self.state_bumper = True

            if self.state != 2:  # Wasn't in obstacle avoidance state before
                self.hit_pt_x = self.current_x  # Store hit point
                self.hit_pt_y = self.current_y
                self.state = 2  # Change to obstacle avoid state

    # Callback for LIDAR
    def callback_sens_scan(self, data_scan):
        self.scan_data = data_scan

        self.scan_regions = [data_scan.ranges[0:19], data_scan.ranges[20:211],
            data_scan.ranges[212:427], data_scan.ranges[428:619],
            data_scan.ranges[621:639]]

        # See an obstacle directly in front and wasn't previously avoiding an obstacle
        # ignores this obstacle if it sees it while returning to the centre line (may catch
        # while turning)
        if data_scan.ranges[319] == 0 and self.state != 2 and self.flag_returning == False:
            self.hit_pt_x = self.current_x  # Store hit point
            self.hit_pt_y = self.current_y
            self.state = 2  # Change to obstacle avoid state


    ### DEFINE ACTIONS ###
    # Follow centre line action
    def act_follow_cline(self):
        cmd_to_line = Twist()

        # What side of line
        if self.current_y > 0.05:  # On left of line by a decent distance
            self.flag_returning = True
            # Turn towards line
            if self.current_angle < 265 or self.current_angle > 275:
                cmd_to_line.angular.z = -self.speed_lineup_ang
            else:  # Drive towards line
                cmd_to_line.angular.z = 0
                cmd_to_line.linear.x = self.speed_lineup_lin

            self.cmd_vel_twist.publish(cmd_to_line)

        elif self.current_y < -0.05:  # On right of line by a decent distance
            self.flag_returning = True
            # Turn towards line
            if self.current_angle < 85 or self.current_angle > 95:
                cmd_to_line.angular.z = self.speed_lineup_ang
            else:  # Drive towards line
                cmd_to_line.linear.x = self.speed_lineup_lin

            self.cmd_vel_twist.publish(cmd_to_line)

        elif self.current_angle > 2 and self.current_angle < 180:  # On line, turn to front cw
            if self.current_angle < 10:  # Swerve in
                cmd_to_line.linear.x = 0.8
                cmd_to_line.angular.z = -0.1
            else:  # Turn in
                self.flag_returning = True
                cmd_to_line.angular.z = -self.speed_lineup_ang

            self.cmd_vel_twist.publish(cmd_to_line)

        elif self.current_angle < 358 and self.current_angle > 180:  # On line, turn to front ccw
            if self.current_angle > 350:  # Swerve in
                cmd_to_line.linear.x = 0.8
                cmd_to_line.angular.z = 0.1
            else:  # Turn in
                self.flag_returning = True
                cmd_to_line.angular.z = self.speed_lineup_ang

            self.cmd_vel_twist.publish(cmd_to_line)

        else:  # Currently turned to front and on centre line, move forward
            self.flag_returning = False
            self.helper_drive_forward()

    # Wall avoid action
    def act_avoid(self):
        # Count times past hit point when returning around obstacle
        if self.flag_changed_dir == True and (abs(self.current_y) < abs(self.hit_pt_y) + 0.2) and (abs(self.current_y) > abs(self.hit_pt_y) - 0.2) \
                and (abs(self.current_x) < abs(self.hit_pt_x) + 0.2) and (abs(self.current_x) > abs(self.hit_pt_x) - 0.2):
            self.flag_passed_hit = True

        # If back on centre line, ahead of hit point and not passing for the first time on
        # turning around, exit obstacle avoidance
        if (self.current_y < 0.05 and self.current_y > -0.05) and self.current_x > (self.hit_pt_x + 0.25) \
                and not(self.flag_changed_dir == True and self.flag_passed_hit == False):
            self.flag_returning = True
            self.flag_changed_dir = False  # Reset flags
            self.flag_passed_hit = False
            self.state = 1

        # If gone 2.5 squares back beyond hit point, likely turned wrong way at end
        # Flag ensures direction not changed again
        elif self.current_x < (self.hit_pt_x - 2.5) and self.flag_changed_dir == False:
            # Turn back towards end, change obstacle avoidance direction and go
            #rospy.loginfo('Threshold hit')
            if self.current_angle < 358 and self.current_angle > 2:  # Not yet facing forwards
                self.flag_returning = True  # Don't let LiDAR exit turn
                cmd_turn_front = Twist()
                cmd_turn_front.angular.z = 2
                self.cmd_vel_twist.publish(cmd_turn_front)

            else:
                self.flag_changed_dir = True  # Set flag to say direction changed
                self.which_dir = -self.which_dir  # Change obstacle avoid direction

        # React with bumper or LIDAR
        elif self.state_bumper == True:
            self.helper_bumper(self.which_dir)

        else:
            # React with LiDAR
            # Lists allow use of the same LiDAR code for both directions
            self.helper_lidar()

    ### Define helper functions ###
    # Drive forward
    def helper_drive_forward(self):
        cmd_twist = Twist()
        cmd_twist.linear.x = self.speed_base_lin

        self.cmd_vel_twist.publish(cmd_twist)

    # Define reaction to seeing objects in the LiDAR
    # ind_dir = array of indicies
    def helper_lidar(self):
        cmd_move_avoid = Twist()

        if self.which_dir == 1:  # Go left around object
            ind_dir = [4, 3, 2, 1, 0]
        else:  # Go right around object
            ind_dir = [0, 1, 2, 3, 4]

        if max(self.scan_regions[ind_dir[0]] + self.scan_regions[ind_dir[1]] + self.scan_regions[ind_dir[2]] + self.scan_regions[ind_dir[3]] +
               self.scan_regions[ind_dir[4]]) == 0.0:
            # See all obstacle
            cmd_move_avoid.linear.x = 0
            cmd_move_avoid.angular.z = self.which_dir*self.speed_lidar_ang
        elif max(self.scan_regions[ind_dir[1]] + self.scan_regions[ind_dir[2]] + self.scan_regions[ind_dir[3]] + self.scan_regions[ind_dir[4]]) == 0:
            # See obstacle everywhere expect right most region
            cmd_move_avoid.linear.x = 0
            cmd_move_avoid.angular.z = self.which_dir*self.speed_lidar_ang
        elif max(self.scan_regions[ind_dir[2]] + self.scan_regions[ind_dir[3]] + self.scan_regions[ind_dir[4]]) == 0:
            # See obstacle in scan_regions 2-4
            cmd_move_avoid.linear.x = 0
            cmd_move_avoid.angular.z = self.which_dir*self.speed_lidar_ang
        elif max(self.scan_regions[ind_dir[3]] + self.scan_regions[ind_dir[4]]) == 0:
            # See obstacle in scan_regions 3-4
            cmd_move_avoid.linear.x = self.speed_lidar_lin
            cmd_move_avoid.angular.z = self.which_dir*(self.speed_lidar_lin/2)
        elif max(self.scan_regions[ind_dir[4]]) == 1 and min(self.scan_regions[ind_dir[4]]) == 0:
            # See obstacle in region 4 a bit
            cmd_move_avoid.linear.x = self.speed_lidar_lin
            cmd_move_avoid.angular.z = self.which_dir*0
        elif max(self.scan_regions[ind_dir[4]]) == 0:
            # See obstacle in region 4 completely
            cmd_move_avoid.linear.x = self.speed_lidar_lin
            cmd_move_avoid.angular.z = self.which_dir*(self.speed_lidar_lin/2)
        elif min(self.scan_regions[ind_dir[4]]) == 1:
            # Sees no obstacle
            cmd_move_avoid.linear.x = self.speed_lidar_lin + 0.1
            cmd_move_avoid.angular.z = self.which_dir*-0.6

        self.cmd_vel_twist.publish(cmd_move_avoid)

    # Defines reaction to hitting the bumper
    # turn_dir - +1 = CCW, -1 = CW
    def helper_bumper(self, turn_dir):
        # Speed varies depending on the bumper hit
        if self.which_bumper == 0:
            self.helper_reverse(turn_dir*self.speed_bumper_ang)
        elif self.which_bumper == 1:
            self.helper_reverse(turn_dir*self.speed_bumper_ang)
        elif self.which_bumper == 2:
            self.helper_reverse(turn_dir*self.speed_bumper_ang)

        # Exit bumper sub-state
        self.state_bumper = False

    # Defines series of movements to get out of bumper reactions
    # turn_speed - +1 = CCW, -1 = CW
    def helper_reverse(self, turn_speed):
        # Initialise variables for bumper response
        cmd_lin_twist = Twist()
        cmd_ang_twist = Twist()

        cmd_lin_twist.linear.x = -self.speed_bumper_lin
        cmd_ang_twist.angular.z = turn_speed

        r = rospy.Rate(10)

        # Reverse for some distance
        for i in range(5):
            self.cmd_vel_twist.publish(cmd_lin_twist)
            r.sleep()

        # Turn in specified direction
        for i in range(7):  # Was 5
            self.cmd_vel_twist.publish(cmd_ang_twist)
            r.sleep()

    ### Define shutdown ###
    def shutdown(self):

        rospy.sleep(1)

if __name__ == '__main__':
    ObstacleAvoidNode()
