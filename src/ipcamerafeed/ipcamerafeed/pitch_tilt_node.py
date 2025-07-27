import rclpy
from rclpy.node import Node
from ipcamerafeed.srv import SetFloat64
import serial

class PTZPitchTiltNode(Node):
    """
    To pitch (DATA IN DEGREES):
        ros2 service call /pitch example_interfaces/srv/SetFloat64 "{data: 30.0}"
    To tilt (DATA IN DEGREES):
        ros2 service call /tilt example_interfaces/srv/SetFloat64 "{data: 30.0}"
    """
    def __init__(self):
        super().__init__('pitch_tilt_node')    

        # Serial port stuff
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200) 

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

        # Assign service for tilting and zoom
        self.pitch_srvs = self.create_service(SetFloat64, 'pitch', self.pitch) 
        self.tilt_srvs = self.create_service(SetFloat64 , 'tilt', self.tilt)

    def pitch(self, request, response):
        if self.ser is not None:
            cmd = f'C {request.data}\r\n'
            self.ser.write(cmd.encode())
            response.data = request.data
        else:
            response.data = -1.0
        return response
    
    def tilt(self, request, response):
        if self.ser is not None:
            cmd = f'P {request.data}\r\n'
            self.ser.write(cmd.encode())
            response.data = request.data
        else:
            response.data = -1.0
        return response

def main(args=None):
    rclpy.init(args=args)
    node = PTZPitchTiltNode()
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