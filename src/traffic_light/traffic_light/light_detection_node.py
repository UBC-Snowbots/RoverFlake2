#!/usr/bin/env python3

"""
Created by: Cameron Basara

Purpose: 
"""

from __future__ import print_function
import rclpy
from rclpy.node import Node
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from concurrent.futures import ThreadPoolExecutor, as_completed 

class BoundBox():
    def __init__(self, x, y, height, width):
        self.x = x
        self.y = y
        self.height = height
        self.width = width
    
    def forward(self):
        return (self.x, self.y, self.height, self.width)

class LightBound(Node):
    def __init__(self):
        super().__init__('light_detection')
        self.bridge = CvBridge()  # setup CVbridge to convert from ImageMessage to CVimage
        self.image_sub = self.create_subscription(Image, "/usb_cam", self.callback, 10)  # subscribe to image_topic topic
        self.imagePubLockIn = self.create_publisher(String, "/color_marker_found", 1) # publish contour of object to topic, with centering ros message
        self.imagePubColObj = self.create_publisher(Image, "/color_objs", 1) # publish raw object of distinct color
        self.imagePubColObjLI = self.create_publisher(Image, "/color_objs_locked", 1) # For locked in topic
        self.imagePubBoundingBox = self.create_publisher(Image, "/bounding_box_contour", 1) # for publsihing the bounding box
       
        # Setup color information
        # color bounds for hsv red
        self.red_lower_bound = [np.array([0, 150, 50], dtype="uint8"), np.array([170, 150, 50], dtype="uint8")]
        self.red_upper_bound = [np.array([10, 255, 255], dtype="uint8"), np.array([180, 255, 255], dtype="uint8")]

        # color bounds for hsv blue
        self.blue_lower_bound = [np.array([100, 150, 50], dtype="uint8")]
        self.blue_upper_bound = [np.array([140, 255, 255], dtype="uint8")]

        self.red = {"name": "red", "lower": self.red_lower_bound, "upper": self.red_upper_bound}
        self.blue = {"name": "blue", "lower": self.blue_lower_bound, "upper": self.blue_upper_bound}

        # Initialize color frontier
        self.frontier = [self.red, self.blue] # TODO: Add IR compatability for ptz cameras
        self.lockin = None # For lockin 

    def create_mask(self, image, lower, upper):
        if len(lower) > 1:
            mask1 = cv2.inRange(image, lower[0], upper[0])
            mask2 = cv2.inRange(image, lower[1], upper[1])
            return cv2.bitwise_or(mask1, mask2)
        else:
            return cv2.inRange(image, lower[0], upper[0])
    
    def detect_color(self, name, lower, upper, hsv_image):
        mask = self.create_mask(hsv_image, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            return name, contours, True
        return name, None, False
    
    def image_color_encoding(self, image):
        if image is None:
            return "Image is None"
    
        if len(image.shape) == 2:
            return "mono8"  # Grayscale image
        
        if len(image.shape) == 3:
            _, _, channels = image.shape
            
        if channels == 1:
            return "mono8"  # Grayscale image
        elif channels == 3:
            return "bgr8"  # BGR image
        elif channels == 4:
            return "bgra8"  # BGRA image
        else:
            return "Unknown encoding"
    


    def track_lockin_color(self, data):
        while self.lockin:
            try:
                # Test
                # self.get_logger().info("trackin")
                # Init image again
                cvImage = self.bridge.imgmsg_to_cv2(data)  # convert image to CV image
                cvImage = cv2.resize(cvImage, (800, 600))  # resize the image
                cvImage = cv2.cvtColor(cvImage, cv2.COLOR_BGR2HSV)
                

                # Init lockin color
                current_color = self.lockin[0]
                lmsg = f"TRUE : Detected {current_color['name']} at {self.get_clock().now().to_msg().sec}"

                # Publish message to topic
                self.imagePubLockIn.publish(String(data=lmsg))

                # Check if color is still in image
                _, _, state = self.detect_color(current_color["name"], current_color["lower"], current_color["upper"], cvImage)

                # self.get_logger().info(f"State:{state}")

                # Get camera feed in while loop
                obj_img = cv2.cvtColor(cvImage, cv2.COLOR_HSV2RGB)
                obj_img = self.bridge.cv2_to_imgmsg(obj_img, encoding='bgr8')
                self.imagePubColObjLI.publish(obj_img)

                if not state:
                    lmsg = 'FALSE'
                    self.imagePubLockIn.publish(String(data=lmsg))
                    self.lockin = None

            except CvBridgeError as e:
                self.get_logger().error(str(e))
            except Exception as e:
                self.get_logger().error(f"Unexpected error: {str(e)}")


    def callback(self, data):  # runs every time an ImageMessage is uploaded on the topic
        try:
            # Initialize image 
            cvImage = self.bridge.imgmsg_to_cv2(data)  # convert image to CV image
            cvImage = cv2.resize(cvImage, (800,600))  # resize the image
            cvImage = cv2.cvtColor(cvImage, cv2.COLOR_BGR2HSV) # convert image to hsv
            contourImage = None # Image for countours
            # self.get_logger().info('Received an image')
            
            # Get info
            (height, width) = cvImage.shape[:2]
            # self.get_logger().info(f'w:{width} h:{height}')

            # Initialize multithreading with ThreadPoolExecuter to handle multiple threads (colors)
            with ThreadPoolExecutor(max_workers=len(self.frontier)) as executor:
                # Submit colors to executor to simultaneously look for colors in the camera feed
                future_to_color = {executor.submit(self.detect_color, cr["name"], cr["lower"], cr["upper"], cvImage): cr for cr in self.frontier}
                
                # Process results as they are completed
                for future in as_completed(future_to_color):
                    try:
                        color_name, contours, _ = future.result()
                        color_list = [color["name"] for color in self.frontier]
                        # self.get_logger().info(f"Number of workers in executor before submission: {len(executor._threads)} | Colours Remaining: {color_list}")
                        
                        if contours:
                            self.get_logger().info(f"Definitive contour found for color: {color_name} at {self.get_clock().now().to_msg().sec}")
                            
                            contourImage = cvImage.copy()
                            grayImage = cv2.cvtColor(cvImage, cv2.COLOR_BGR2GRAY)
                            grayImage = cv2.cvtColor(grayImage, cv2.COLOR_GRAY2BGR)
                            
                            colorMask = cv2.bitwise_and(cvImage, cvImage, mask=mask)
                            grayMask = cv2.bitwise_and(grayImage, grayImage, mask=cv2.bitwise_not(mask))
                            highlightedImage = cv2.add(colorMask, grayMask)
                            for contour in contours:
                                cv2.drawContours(contourImage, [contour], -1, (0, 255, 0), thickness=cv2.FILLED)
                    
                            # If contours found, we want to draw a box around them
                            # for contour in contours:
                            #     # x, y, w, h = cv2.boundingRect(contour)
                            #     # bbox_msg = BoundBox(x, y, w, h)
                            #     # self.get_logger().info(f'Bounding box info: x: {x} y: {y} h: {h} w:{w}')
                            #     # cv2.rectangle(cvImage, (x, y), (x+w, y+h), (0, 255, 0), 2)
                            #     # cv2.putText(cvImage, color_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                            #     contourImage =  cv2.drawContours(cvImage, [contour], -1, (0, 255, 0), thickness=cv2.FILLED)
                                

                            # Convert the image with filled contours to a ROS 2 image message
                            contourImage = cv2.cvtColor(contourImage, cv2.COLOR_HSV2RGB)
                            contourImageMsg = self.bridge.cv2_to_imgmsg(contourImage, encoding='bgr8')

                            # Publish the image
                            self.imagePubBoundingBox.publish(contourImageMsg)

                            highlightedImageMsg = self.bridge.cv2_to_imgmsg(highlightedImage, encoding='bgr8')
                            self.imagePubColObj.publish(highlightedImageMsg)
                            # Convert image from hsv to bgr and publish video feed
                            # self.get_logger().info("Not l ocked in")
                            # obj_img = cv2.cvtColor(cvImage, cv2.COLOR_HSV2BGR)
                            # obj_img = self.bridge.cv2_to_imgmsg(obj_img, encoding='bgr8')
                            # self.imagePubColObj.publish(obj_img)

                            # Remove the detected color from the frontier, add locked in color
                            if self.lockin is None:
                                self.lockin = [cr for cr in self.frontier if cr["name"] == color_name]
                                self.frontier = [cr for cr in self.frontier if cr["name"] != color_name]

                            # Start a new thread to handle the lock-in color tracking
                            # lockin_thread = ThreadPoolExecutor(max_workers=1)
                            # executor.submit(self.track_lockin_color, data)
                            break                  
                        
                        # Shutdown the executor as soon as a definitive contour is found
                        executor.shutdown(wait=False)
                        break
                    
                    except Exception as e:
                        self.get_logger().error(f"Error in thread: {str(e)}")

    
        except CvBridgeError as e:
            self.get_logger().error(str(e))


def main(args=None):
    rclpy.init(args=args)
    Node = LightBound()
    try:
        rclpy.spin(Node)  # loop until shutdown
    except KeyboardInterrupt:
        Node.get_logger().info('Shutting down')
    finally:
        Node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
