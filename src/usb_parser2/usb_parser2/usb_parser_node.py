"""
Created by: Cameron Basara
Date Created: 2/11/2024
Purpose: To retrieve data from a usb-drive, and intepret the data regarding a CIRC 2024 Search and Rescue Task
Log file format: [ISO 8601 Timestamp] [Severity] Cell [Number] Diagnostic [OK/FAIL] [message]
                  2023-12-12T23:45:06.3743Z WARN Cell 2 Diagnostic FAIL Short Detected.
"""
#!/usr/bin/env python3

import os
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class usbParserNode(Node):
    def __init__(self):
        super().__init__("usb_parser")
        self.publisher = self.create_publisher(String, '/usb_parsed_data', 30)
        self.get_logger().info('Running Diagnostic...')

    def message_callback(self, data):
         # This method publishes the parsed data to 'usb_parsed_data'
        msg = String()
        msg.data = data
        self.publisher.publish(msg)
        self.get_logger().info(f'{msg.data}')

def usbParser():
    # Retrieve data from file
    script_path = os.path.dirname(__file__)  # Gets the directory where the script is located
    file_path = os.path.join(script_path, '../logs/lander.log')  # Navigate to the target file
    data =  r'%s' % file_path
    
    with open(data) as f:
        # Declare diagnostic var
        message = []

        # Iterate lines in f
        for line in f:
            hold = re.search("FAIL", line) 
            if hold is not None:
                message.append(line[:-1])
        
        return str(message)

def main(args=None):
    # Initialize our node, 
    rclpy.init(args=args)
    node = usbParserNode()

    # Publish callback
    try:
        data = usbParser()
        # data = "Test data"
        node.message_callback(data)
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



