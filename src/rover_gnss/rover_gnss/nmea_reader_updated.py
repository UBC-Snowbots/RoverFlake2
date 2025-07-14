import serial
import time
import sys
import pynmea2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


DEVICE = '/dev/ttyACM0'
#DEVICE = '/mnt/shared/RoverFlake2/serial'
BAUD_RATE = 38400

# Serial device
class SerialDevice:
    def __init__(self, device, baud_rate):
        self.device = device
        self.baud_rate = baud_rate
        self.timeout = 1

# Class to read NMEA data   
class NMEAReader(Node):
    def __init__(self):
        # ROS2 part
        super().__init__('nmea_reader')
        self.publisher = self.create_publisher(NavSatFix, 'gnss_fix', 10)
        self.i = 0 # magic variable :3

        # Serial part
        serialdevice = SerialDevice(DEVICE, BAUD_RATE)
        self.openSerial(serialdevice)        
        self.timer = self.create_timer(0.5, self.readSerial)

        # 
        self.f = open('nmealog.txt','a')

    # Timer callback for ROS2
    def timer_callback(self):
        msg = NavSatFix()
        msg.latitude = 0

    
    # Take the serial device and try to open the port, kill self otherwise 
    def openSerial(self, SerialDevice):
        SerialDevice = SerialDevice
        try:
            self.ser = serial.Serial(SerialDevice.device, SerialDevice.baud_rate, timeout=SerialDevice.timeout)
            self.ser.flushInput()
            self.ser.flushOutput()
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            return None
        except FileNotFoundError:
            print("[NMEA/Serial] !!! Serial port not found: {}\nTerminating...".format(DEVICE))
            sys.exit(1)
    
    # Continuously read the serial port and parse NMEA data
    def readSerial(self):
        scanning = True
        if self.ser.isOpen():
            print("[NMEA/Serial] Serial open, trying read")
            while scanning and rclpy.ok():
                try:
                    line = self.ser.readline().decode('ascii', errors='replace')
                    # print(f'[READ] line: {line[3:7]}')
                    
                    # read the first 6 characters to determine the type of NMEA message
                    self.f.write(line)

                    if line[3:6] == 'GGA':
                        print(line)
                        try:
                            natsavfix = self.parseNMEA(line)
                            if type(natsavfix) is not NavSatFix:
                                print("[NMEA/Reader] Couldn't parse message!")
                            else:
                                self.publishPosition(natsavfix)
                        except pynmea2.ParseError as e:
                            print("[NMEA/Reader] ERROR: PyNMEA error: {}".format(e))
                    
                except KeyboardInterrupt:
                    print("[NMEA/Reader] KeyboardInterrupt received, killing myself")
                    break
        else:
            self.get_logger().error("[NMEA/Reader] Serial port is closed, cannot read data")
            print(time.strftime("[NMEA/Reader] (%H:%M:%S) ", time.localtime()) + "Serial closed, this shouldn't happen")
    
    # Parse NMEA data
    def parseNMEA(self, line):
        msg = pynmea2.parse(line)
        if not isinstance(msg, pynmea2.types.talker.GGA):
            self.get_logger().error("[NMEA/Parser] ERROR: Received a non-GGA message, this shouldn't happen")
            print("[NMEA/Parser] ERROR: Received a non-GGA message, this shouldn't happen")
            return None
        else:
            navsat_fix = NavSatFix()
            navsat_fix.latitude = msg.latitude
            navsat_fix.longitude = msg.longitude
            # navsat_fix.altitude = msg.altitude
            return navsat_fix


    # Publish NMEA data
    def publishPosition(self, navfixobj):
        if type(navfixobj) is not NavSatFix:
            self.get_logger().error("[NMEA/Publisher] ERROR: Received a non-NavSatFix message, this shouldn't happen")
            print("[NMEA/Publisher] ERROR: Received a non-NavSatFix message, this shouldn't happen")
            pass
        else:
            self.get_logger().info('Publishing NMEA data...')
            print(time.strftime("[NMEA] (%H:%M:%S) ", time.localtime()) + "Lat: {} Lon: {}, Alt: {}".format(navfixobj.latitude, navfixobj.longitude, navfixobj.altitude))
            self.publisher.publish(navfixobj)
        return None

    def closeSerial(self, ser=None):
        if ser is None:
            ser = self.ser
        if not ser.isOpen():
            print("[NMEA/Serial] !! Serial port is already closed")
        else:
            ser.flushInput()
            ser.flushOutput()
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            ser.close()
            if ser.isOpen():
                print("[NMEA] !!! Serial port is still open")
            else:
                print("[NMEA] Serial port closed successfully")
            del ser
            sys.exit(0)
        
        sys.exit(0) 

def main():
    rclpy.init()
    reader = NMEAReader()
    rclpy.spin(reader)
    reader.closeSerial()
    rclpy.shutdown()
    sys.exit(0)
