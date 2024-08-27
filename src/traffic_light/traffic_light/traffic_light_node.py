#!/usr/bin/env python

#
# This script reads images from a ROS topic,
# converts them and elaborates to detect red and 
# blue LEDs (semaphore), eventually sending 
# encoded strings to the redmask_detection_topic topic
#
# script by Andrea Fioroni - andrifiore@gmail.com
# GitHub repo: https://github.com/isarlab-department-engineering/ros-traffic-light-detection
#

"""
Made some adjustements to better suit the competition task - Cameron
"""

from __future__ import print_function
import rclpy
from rclpy.node import Node
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class TrafficLight(Node):

    def __init__(self):
        super().__init__('red_mask_detection')
        self.bridge = CvBridge()  # setup CVbridge to convert from ImageMessage to CVimage
        self.image_sub = self.create_subscription(
            Image, "/usb_cam", self.callback, 10)  # subscribe to image_topic topic
        self.imagePubRed = self.create_publisher(
            Image, "red_mask_topic", 1)
        # self.imagePubBlue = self.create_publisher(
            # Image, "blue_mask_topic", )
        
        self.controlPub = self.create_publisher(
            String, "redmask_detection_topic", 10)

        # semaphore variables
        self.redSemaphore = 0
        self.blueSemaphore = 0

        # # color boundaries (bgr format) -> BLUE
        # self.Rlower = np.array([110, 50, 50], dtype="uint8")  # lower boundaries
        # self.Rupper = np.array([130, 255, 255], dtype="uint8")  # upper boundaries

        # # color boundaries (bgr format) -> RED
        # self.Blower = np.array([20, 200, 20], dtype="uint8")  # lower boundaries
        # self.Bupper = np.array([50, 255, 50], dtype="uint8")  # upper boundaries
        
        # Colour bounds for hsv red
        self.red_lower_bound1 = np.array([0, 150, 50], dtype="uint8")
        self.red_upper_bound1 = np.array([10, 255, 255], dtype="uint8")
        self.red_lower_bound2 = np.array([170, 150, 50], dtype="uint8")
        self.red_upper_bound2 = np.array([180, 255, 255], dtype="uint8")

        # Colour bounds for hsv blue
        self.blue_lower_bound = np.array([100, 150, 50], dtype="uint8")
        self.blue_upper_bound = np.array([140, 255, 255], dtype="uint8")

        # rows and columns boundaries
        # suppose you will always find the semaphore
        # on the top right corner of the image 
        self.imin = 0
        self.imax = 25
        self.jmin = 0
        self.jmax = 155
        self.step = 1

    def callback(self, data):  # runs every time an ImageMessage is uploaded on the topic
        try:
            cvImage = self.bridge.imgmsg_to_cv2(data)  # convert image to CV image
            cvImage = cv2.resize(cvImage, (800,600))  # resize the image
            cvImage = cv2.cvtColor(cvImage, cv2.COLOR_BGR2HSV) # convert image to hsv

            self.get_logger().info('Received an image')
            
            (height, width) = cvImage.shape[:2]
             
            self.get_logger().info(f'w:{width} h:{height}')

            # color detection
            # generate blue and red masks and use them to filter the image
            # redMask / blueMask images will display only red / blue pixels

            # I changed the masks to fit the HSV format as HSV is better suited for this task
            rmask1 = cv2.inRange(cvImage, self.red_lower_bound1, self.red_upper_bound1)  # red boundaries mask
            rmask2 = cv2.inRange(cvImage, self.red_lower_bound2, self.red_upper_bound2)  # red boundaries mask
            rmask = cv2.bitwise_or(rmask1, rmask2)   
            bmask = cv2.inRange(cvImage, self.blue_lower_bound, self.blue_upper_bound)  # blue boundaries mask

            redResult = cv2.bitwise_and(cvImage, cvImage, mask=rmask)  # red filtered image
            blueResult = cv2.bitwise_and(cvImage, cvImage, mask=bmask)  # blue filtered image

            combinedMask = cv2.bitwise_or(rmask, bmask)
            combinedRes = cv2.bitwise_and(cvImage, cvImage, mask=combinedMask)
           
            

            (rh, rw) = combinedRes.shape[:2] 
            self.get_logger().info(f'rw:{rw} rh:{rh}')

            # for i in range(0, rw):
            # redResult = cv2.addWeighted(cvImage, 0.7, rmask, 0.5, 0) 

            # # wait for blue semaphore to restart
            # if self.redSemaphore == 1:  # look for blue semaphore only after finding a red one
            #     for i in range(self.imin, width, self.step * 2):
            #         for j in range(self.jmin, height, self.step * 2):
            #             if bmask[i][j] > 0:  # if that pixel is blue
            #                 self.redSemaphore = 0  # reset redSemaphore
            #                 self.blueSemaphore = 1  # found a blueSemaphore
            # if self.blueSemaphore == 1:
            #     self.controlPub.publish(String(data="GGG"))  # send blue encoded string to traffic_light_detection
            #     self.blueSemaphore = 0

            # self.redSemaphore = 0  # reset redSemaphore each time
            # for i in range(self.imin, width, self.step):  # look for a red semaphore
            #     for j in range(self.jmin, height, self.step):
            #         if rmask[i][j] > 0:  # if that pixel is red
            #             self.redSemaphore = 1  # found a redSemaphore
            # if self.redSemaphore == 1:
            #     self.get_logger().info('FOUND RED')
            #     self.controlPub.publish(String(data="RRR"))  # send red encoded string to traffic_light_detection

            # # redundant check: every time, if no redSemaphore is found (and not even a blue one)
            # # send blue encoded string to rospibot_network
            # if self.redSemaphore == 0:
            #     self.controlPub.publish(String(data="GGG"))    # wait for blue semaphore to restart
            # if self.redSemaphore == 1:  # look for blue semaphore only after finding a red one
            #     for i in range(self.imin, width, self.step * 2):
            #         for j in range(self.jmin, height, self.step * 2):
            #             if bmask[i][j] > 0:  # if that pixel is blue
            #                 self.redSemaphore = 0  # reset redSemaphore
            #                 self.blueSemaphore = 1  # found a blueSemaphore
            # if self.blueSemaphore == 1:
            #     self.controlPub.publish(String(data="GGG"))  # send blue encoded string to traffic_light_detection
            #     self.blueSemaphore = 0

            # self.redSemaphore = 0  # reset redSemaphore each time
            # for i in range(self.imin, width, self.step):  # look for a red semaphore
            #     for j in range(self.jmin, height, self.step):
            #         if rmask[i][j] > 0:  # if that pixel is red
            #             self.redSemaphore = 1  # found a redSemaphore
            # if self.redSemaphore == 1:
            #     self.get_logger().info('FOUND RED')
            #     self.controlPub.publish(String(data="RRR"))  # send red encoded string to traffic_light_detection

            # # redundant check: every time, if no redSemaphore is found (and not even a blue one)
            # # send blue encoded string to rospibot_network
            # if self.redSemaphore == 0:
            #     self.controlPub.publish(String(data="GGG"))

            cv2.rectangle(redResult, (self.jmin, self.imin), (height, width), (30, 30, 255), 1)
            self.imagePubRed.publish(self.bridge.cv2_to_imgmsg(combinedRes))  # publish the red masked image on the red_mask_topic topic

        except CvBridgeError as e:
            self.get_logger().error(str(e))


def main(args=None):
    rclpy.init(args=args)
    traffic_light_node = TrafficLight()
    try:
        rclpy.spin(traffic_light_node)  # loop until shutdown
    except KeyboardInterrupt:
        traffic_light_node.get_logger().info('Shutting down')
    finally:
        traffic_light_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
