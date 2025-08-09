#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        # Read from serial port
        self.serial_port = serial.Serial(
            '/dev/ttyACM0', 9600, timeout=1
        )

        self.pub_ = self.create_publisher(String, 'gas_sensor_data', 10)
        self.timer = self.create_timer(0.1, self.read_serial) # spin at 10 hz

    def read_serial(self):
        if self.serial_port.in_waiting > 0:
            data = self.serial_port.readline().decode('utf-8').strip()
            msg = String()
            msg.data = data
            self.pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()