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
        self.ser
        scanning = True
        while scanning and rclpy.ok():
            try:
                if self.ser.isOpen():
                    print("[NMEA/Serial] Serial open, trying read")
                    line = self.ser.readline().decode('ascii', errors='replace')                
                    #timestamp = f"{time.localtime().tm_hour:time.localtime().tm_min:time.localtime().tm_sec}"
                    #print(f" ((({timestamp}))) {line}")
                    if line.startswith('$GP'):
                        if line.startswith('$GNEBP'): # Special if statement because pynmea2 is scared of EBP messages
                            break
                        try:
                            print(line)
                            
                            natsavfix = self.parseNMEA(line)
                            if type(natsavfix) is NavSatFix:
                            	self.publishPosition(natsavfix)
                            else:
                                print("[NMEA/Reader] Non NSF message, skipping...")
                        except pynmea2.ParseError as e:
                            #print("[M+] Parse error: {}".format(e)) # Muting errors, should redirect them into ROS log later
                            pass
                else:
                    print(time.strftime("[NMEA/DEBUG] (%H:%M:%S) ", time.localtime()) + "CLOSED")
            except KeyboardInterrupt:
                break
    
    # Parse NMEA data
    def parseNMEA(self, line):
        msg = pynmea2.parse(line)
        if isinstance(msg, pynmea2.types.talker.GGA):
            print(f"Lat {msg.latitude}; Lon {msg.longitude}; Alt {msg.altitude}")
            navsat_fix = NavSatFix()
            navsat_fix.latitude = msg.latitude
            navsat_fix.longitude = msg.longitude
            navsat_fix.altitude = msg.altitude
            return navsat_fix
        else:
            print("[NMEA/Parser] !!! Non-GGA message: {}".format(msg))
            return None

    # Publish NMEA data
    def publishPosition(self, navfixobj):
        if type(navfixobj) is not NavSatFix:
            print("[NMEA/Publisher] !!! Received a non-NavSatFix message")
            pass
        else:
            self.get_logger().info('Publishing NMEA data...')
            print(time.strftime("[NMEA] (%H:%M:%S) ", time.localtime()) + "Lat: {}{} Lon: {}{}, Alt: {}".format(msg.latitude, msg.lat_dir, msg.longitude, msg.lon_dir, msg.altitude))
            self.publisher.publish(navfixobj)
        return None

    def closeSerial(self, ser=None):
        if ser is None:
            ser = self.ser
        if not ser.isOpen():
            print("[M+] !! Serial port is already closed")
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
