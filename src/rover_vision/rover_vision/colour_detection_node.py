#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header, String, Bool
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import Marker

class ColourDetectionNode(Node):
    def __init__(self):
        super().__init__('enhanced_color_detector')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()

        # TF2 for coordinate transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Parameters
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('detection_threshold', 0.05)  # Min area percentage, needs to be fiddled with    
        
        # Color definitions 
        self.color_definitions = {
            'red': {
                'lower': [np.array([0, 150, 50]), np.array([170, 150, 50])],
                'upper': [np.array([10, 255, 255]), np.array([180, 255, 255])],
                'bgr': (0, 0, 255)
            },
            # 'blue': {
            #     'lower': [np.array([100, 150, 50])],
            #     'upper': [np.array([140, 255, 255])],
            #     'bgr': (255, 0, 0)
            # },
            # 'ir_white': {
            #     'lower': [np.array([0, 0, 200])],  
            #     'upper': [np.array([180, 30, 255])], 
            #     'bgr': (255, 255, 255)
            # }
        }   
        
        # State management
        self.color_horizon = list(self.color_definitions.keys()) # sheesh
        self.locked_color = None
        self.target_reached = False
        
        # Publishers
        self.detection_pub = self.create_publisher(
            PointStamped, '/detected_light_position', 10)
        self.status_pub = self.create_publisher(
            String, '/color_detection_status', 10)
        self.locked_color_pub = self.create_publisher(
            String, '/locked_color', 10)
        self.visualization_pub = self.create_publisher(
            Marker, '/color_detection_marker', 10)
        self.annotated_image_pub = self.create_publisher(
            Image, '/color_detection_annotated', 10)
        
        # Subscribers
        self.rbg_image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 30)
        # self.ir_image_sub = self.create_subscription(
        #     Image, '/camera/infra1/image_rect_raw', self.image_callback, 30)
        self.target_reached_sub = self.create_subscription(
            Bool, '/target_reached', self.target_reached_callback, 10)
        
        # Camera parameters (need to update to ptz cam)
        self.camera_fov_horizontal = 69.4 * (np.pi / 180)  # realsense deg in rads
        self.camera_fov_vertical = 42.5 * (np.pi / 180)   # from googles
        
        self.get_logger().info(f'Color detector initialized with horizon: {self.color_horizon}')
    
    def target_reached_callback(self, msg):
        """Called when rover reaches a light target"""
        if msg.data and self.locked_color:
            self.get_logger().info(f'Target reached for color: {self.locked_color}')
            # Remove color from horizon
            if self.locked_color in self.color_horizon:
                self.color_horizon.remove(self.locked_color)
            self.locked_color = None
            self.get_logger().info(f'Updated horizon: {self.color_horizon}')

    def create_mask(self, hsv_image, color_def):
        """Create mask for color detection"""
        masks = []
        for i in range(len(color_def['lower'])):
            mask = cv2.inRange(hsv_image, color_def['lower'][i], color_def['upper'][i])
            masks.append(mask)
        
        if len(masks) > 1:
            return cv2.bitwise_or(*masks)
        return masks[0]

    def get_bearing_to_pixel(self, x, image_width):
        """Convert pixel x-coordinate to bearing angle"""
        # Normalize pixel position to [-1, 1]
        normalized_x = (2.0 * x / image_width) - 1.0
        # Convert to angle
        bearing = normalized_x * (self.camera_fov_horizontal / 2.0)
        return bearing

    def detect_colours(self, hsv_image):
        """Detect all colours in the current horizon"""
        detections = []
        height, width = hsv_image.shape[:2]
        min_area = width * height * self.get_parameter('detection_threshold').value
        
        for color_name in self.color_horizon:
            if color_name not in self.color_definitions:
                continue
                
            color_def = self.color_definitions[color_name]
            mask = self.create_mask(hsv_image, color_def)
            
            # reduce noise
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < min_area:
                    continue
                
                # Get centroid
                M = cv2.moments(contour)
                if M["m00"] == 0:
                    continue
                    
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Calculate bearing
                bearing = self.get_bearing_to_pixel(cx, width)
                
                detections.append({
                    'color': color_name,
                    'centroid': (cx, cy),
                    'area': area,
                    'bearing': bearing,
                    'contour': contour
                })
        
        return detections

    def publish_detection(self, detection, image_shape):
        """Publish the detection as a point in the robot's frame"""
        # Create point in camera frame (bearing only, no distance)
        point_stamped = PointStamped()
        point_stamped.header.stamp = self.get_clock().now().to_msg()
        point_stamped.header.frame_id = self.get_parameter('camera_frame').value
        
        # Set a nominal distance for visualization
        # In practice, we'll use depth camera or fixed height assumption
        nominal_distance = 5.0  # meters
        point_stamped.point.x = nominal_distance
        point_stamped.point.y = -nominal_distance * np.tan(detection['bearing'])
        point_stamped.point.z = 0.0
        
        try:
            # Transform to base frame
            transform = self.tf_buffer.lookup_transform(
                self.get_parameter('base_frame').value,
                self.get_parameter('camera_frame').value,
                rclpy.time.Time())
            
            # Transform the NOMINAL position of cam frame to base frame 
            point_base = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
            self.detection_pub.publish(point_base) # to detected light position
            
            # Publish visualization marker
            self.publish_marker(point_base, detection['color'])
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF error: {e}')

    def publish_marker(self, point_stamped, color_name):
        """Publish visualization marker"""
        marker = Marker()
        marker.header = point_stamped.header
        marker.ns = "color_detectiaon"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point_stamped.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        
        # Set color
        color_bgr = self.color_definitions[color_name]['bgr']
        marker.color.r = color_bgr[2] / 255.0
        marker.color.g = color_bgr[1] / 255.0
        marker.color.b = color_bgr[0] / 255.0
        marker.color.a = 1.0
        
        marker.lifetime.sec = 1
        self.visualization_pub.publish(marker)

    def annotate_image(self, cv_image, detections):
        """Draw annotations on the image"""
        annotated = cv_image.copy()
        
        for detection in detections:
            # Draw contour
            cv2.drawContours(annotated, [detection['contour']], -1, 
                           self.color_definitions[detection['color']]['bgr'], 2)
            
            # Draw centroid
            cx, cy = detection['centroid']
            cv2.circle(annotated, (cx, cy), 10, 
                      self.color_definitions[detection['color']]['bgr'], -1)
            
            # Add text
            text = f"{detection['color']}: {np.degrees(detection['bearing']):.1f}°"
            cv2.putText(annotated, text, (cx - 50, cy - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # Show status
        status_text = f"Horizon: {', '.join(self.color_horizon)}"
        if self.locked_color:
            status_text += f" | Locked: {self.locked_color}"
        cv2.putText(annotated, status_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return annotated

    def image_callback(self, msg):
        """Process incoming images"""
        try:
            # Convert to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Detect colours
            detections = self.detect_colours(hsv_image)
            
            # Process detections
            if detections:
                # Sort by area (largest first)
                detections.sort(key=lambda x: x['area'], reverse=True)
                
                # Lock onto color if not locked
                if not self.locked_color:
                    self.locked_color = detections[0]['color']
                    self.locked_color_pub.publish(String(data=self.locked_color))
                    self.get_logger().info(f'Locked onto color: {self.locked_color}')
                
                # Publish detection for locked color
                for detection in detections:
                    if detection['color'] == self.locked_color:
                        self.publish_detection(detection, hsv_image.shape)
                        status_msg = f"Tracking {self.locked_color} at bearing: {np.degrees(detection['bearing']):.1f}°"
                        self.status_pub.publish(String(data=status_msg))
                        break
            
            # Publish annotated image
            annotated = self.annotate_image(cv_image, detections)
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
            self.annotated_image_pub.publish(annotated_msg)
        
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ColourDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()