#! /usr/bin/env python

from __future__ import print_function
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class TakePhoto:
    def __init__(self):
        # Initialise node
        rospy.init_node('TakePhotoNode', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        self.bridge = CvBridge()
        self.image_received = False

        # Subscribers
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback_take_photo)
        rospy.Subscriber('/cmd_photo', String, self.callback_flag_photo)

        # Instance variables
        self.count_img = 0
        self.flag_take_photo = False
        self.path_save_img = '/home/harvey/compsys726/src/python_nodes/src/Obstacle_pictures' # CHANGE THIS DIRECTORY

        rospy.spin()

    ### DEFINE CALLBACKS ###
    def callback_take_photo(self, data_image):
        if self.flag_take_photo == True:
            # Set picture title
            self.count_img = self.count_img + 1
            img_title = 'Obstacle_' + str(self.count_img) + '.jpg'

            # Convert image to OpenCV format
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data_image, "bgr8")
                # Save image
                cv2.imwrite(self.path_save_img + '/' + img_title, cv_image)

                # Log image name
                rospy.loginfo('Picture taken. Name: ' + img_title)

            except CvBridgeError as e:
                print(e)

            self.flag_take_photo = False

    def callback_flag_photo(self, data_title):
        self.flag_take_photo = True

    ### DEFINE SHUTDOWN ###
    def shutdown(self):
        rospy.sleep(1)

if __name__ == '__main__':
    TakePhoto()
