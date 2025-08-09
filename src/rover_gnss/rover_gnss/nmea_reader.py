import serial
import time
import sys
import pynmea2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from os.path import expanduser


DEVICE = '/dev/ttyACM0'
# DEVICE = 'usb-Emlid_ReachM+_82435519E0ADA406-if02' # hows this for long max
BAUD_RATE = 38400
REACH_LOG = f"{expanduser('~')}/reach_log.txt"

# Serial device
class SerialDevice:
    def __init__(self, device, baud_rate):
        self.device = device
        self.baud_rate = baud_rate
        self.timeout = 1

# Class to read NMEA data   
class NMEAReader(Node):
    def __init__(self, debug=False, altitude=False):

        # control flags
        self.debug = debug
        self.altitude = altitude

        # print flag status
        print("\033[33m", f"[WARN] Debug is enabled, Reach output is saved to {REACH_LOG}", "\033[0m") if self.debug else print("\033[31m", "[WARN] Debug is disabled", "\033[0m")
        print("\033[32m", "[WARN] Altitude publishing is enabled", "\033[0m") if self.altitude else print("\033[31m", "[WARN] Altitude publishing is disabled", "\033[0m")

        # ROS2 part
        super().__init__('nmea_reader')
        self.publisher = self.create_publisher(NavSatFix, 'gnss_fix', 10)        

        # Serial part
        serialdevice = SerialDevice(DEVICE, BAUD_RATE)
        try:
            self.openSerial(serialdevice)        
            self.timer = self.create_timer(0.5, self.readSerial)
        except serial.SerialException as e:
            self.log('e', f"[NMEA/Serial] Couldn't open serial: {e}")
            exit(1)

        # logging Reach output for debugging
        if self.debug:
            self.f = open(REACH_LOG, 'a+')
            self.f.write(f"--- STARTING REACH LOG @ {time.strftime('(%d/%m %H:%M:%S) ---', time.localtime())}\n")

    # Timer callback for ROS2
    def timer_callback(self):
        msg = NavSatFix()
        msg.latitude = 0

    # log function to combine log and print into one cuz clean code
    # types:
    #   e - error
    #   i - info
    #   w - warn
    #   d - debug
    def log(self, type, msg):
        if type == 'e':
            print("\033[31m", self.get_logger().error(msg), "\033[0m") # make it scary red wooooooo
        elif type == 'i':
            self.get_logger().info(msg)
        elif type == 'w':
            print("\033[35m", self.get_logger().warn(msg), "\033[0m") # make it concerning purple
        elif type == 'd':
            print("\033[36m", f"[DEBUG] {repr(msg)}", "\033[0m") # make "Debug Cyan"^(tm)
        else:
            print("[NMEA/Logger]: Unknown msg type, printing as warn")
            print("\033[35m", self.get_logger().warn(msg), "\033[0m") # make it concerning purple
            

    # Take the serial device and try to open the port, kill self otherwise 
    def openSerial(self, SerialDevice):
        try:
            self.ser = serial.Serial(SerialDevice.device, SerialDevice.baud_rate, timeout=SerialDevice.timeout)
            self.ser.flushInput()
            self.ser.flushOutput()
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            return None
        except FileNotFoundError:
            self.log('e', f"[NMEA/Serial] Serial port not found: {DEVICE}, terminating...")
            sys.exit(1)
    
    # Continuously read the serial port and parse NMEA data
    def readSerial(self):
        scanning = True # jsut here in case we decide to code like humans and not monkeys
        if self.ser.isOpen():
            self.log('i', "[NMEA/Serial] Serial open, trying read")
            while scanning and rclpy.ok():
                try:
                    line = self.ser.readline().decode('ascii', errors='replace')
                    
                    # write Reach output to log for debugging
                    self.f.write(line) if self.debug else ''

                    # read the first 6 characters to determine the type of NMEA message
                    if line[3:6] == 'GGA':
                        self.log('d', line)
                        try:
                            natsavfix = self.parseNMEA(line)
                            if type(natsavfix) is not NavSatFix:
                                self.log('w', "[NMEA/Reader] Couldn't parse message!")
                            else:
                                self.publishPosition(natsavfix)
                        except pynmea2.ParseError as e:
                            self.log('e', f"[NMEA/Reader] ERROR: PyNMEA error: {e}")
                    
                except KeyboardInterrupt:
                    self.log('i', "[NMEA/Reader] KeyboardInterrupt received, killing myself")
                    exit(0)
        else:
            self.log('e', "[NMEA/Reader] Serial port is closed, cannot read data")
    
    # Parse NMEA data
    def parseNMEA(self, line):
        msg = pynmea2.parse(line)
        if not isinstance(msg, pynmea2.types.talker.GGA):
            self.log('e', "[NMEA/Parser] ERROR: Received a non-GGA message, this shouldn't happen")
            return None
        else:
            navsat_fix = NavSatFix()
            navsat_fix.latitude = msg.latitude
            navsat_fix.longitude = msg.longitude

            # workaround for when there's no fix and sensors_msgs doesn't get altitude and eats shit
            if type(msg.altitude) == float:
                navsat_fix.altitude = msg.altitude
            else:
                navsat_fix.altitude = 0.0
            
            return navsat_fix


    # Publish NMEA data
    def publishPosition(self, navfixobj):
        if type(navfixobj) is not NavSatFix:
            self.log('w', "[NMEA/Publisher] WARN: Received a non-NavSatFix message, this shouldn't happen")
        else:
            # i am honestly and truly not sorry for this long ass line, fuck you
            print(time.strftime("[NMEA] (%H:%M:%S) ", time.localtime()), f"Lat: {navfixobj.latitude} Lon: {navfixobj.longitude}", f" Alt: {navfixobj.altitude}" if self.altitude else '')
            self.publisher.publish(navfixobj)
        return None

    def closeSerial(self, ser=None):
        if ser is None:
            ser = self.ser
        if not ser.isOpen():
            self.log('w', "[NMEA/Serial] Serial port is already closed")
        else:
            ser.flushInput()
            ser.flushOutput()
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            ser.close()
            if ser.isOpen():
                self.log('w', "[NMEA] Serial port is still open for some reason")
            else:
                self.log('i', "[NMEA] Serial port closed successfully")
            del ser        
        sys.exit(0) 

def main():
    print("oh shit we're going??")
    rclpy.init()
    reader = NMEAReader(debug=True, altitude=True)
    rclpy.spin(reader)
    reader.closeSerial()
    rclpy.shutdown()
    sys.exit(0)

if __name__ == "__main__":
    print("NMEA Reader was not started as a ROS node, aborting...")
    sys.exit(1)
