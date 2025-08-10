#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import serial

class PTZPanTiltNode(Node):
    def __init__(self):
        super().__init__('Pan_tilt_node')    

        # Serial port stuff
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600) 

        # Get parameter values
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # Serial connection
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=1)
            self.get_logger().info('Serial conn for PT')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.ser = None

        self.ptz_sub = self.create_subscription(
            Vector3, '/ptz/control', self.cmd_callback, 10
        ) 

    def cmd_callback(self, msg):
        self.pan(msg.x)
        self.tilt(msg.y)
        
    def pan(self, pan_cmd):
        if self.ser is not None:
            # p = f'0+{pan_cmd*2}' if pan_cmd >= 0 else f'0{pan_cmd*2}'
            # self.ser.write(p.encode('ascii'))
            if pan_cmd == 0:
                # pcmd = 'stop_pan\n'
                pcmd = f'stop_pan\n'
                self.ser.write(pcmd.encode('ascii'))
            else:
                pcmd = f'0+{pan_cmd*2}\n' if pan_cmd > 0 else f'0-{pan_cmd*2}\n'
                self.ser.write(pcmd.encode('ascii'))
            
            # resp = self.ser.read_until(b'\n').decode('ascii') # retrive positional msg
            # self.get_logger().info(f"PAN POSITION: {resp}")
    def tilt(self, tilt_cmd):
        if self.ser is not None:
            # self.get_logger().info(f'tilt {tilt_cmd}')
            tcmd = f'1{tilt_cmd*15}\n'
            self.ser.write(tcmd.encode('ascii'))
            # resp = self.ser.read_until(b'\n').decode('ascii') # retrive positional msg
            # self.get_logger().info(f"TILT POSITION: {resp}")

def main(args=None):
    rclpy.init(args=args)
    node = PTZPanTiltNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down.')
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
            node.get_logger().info("Serial connection closed.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()