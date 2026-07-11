import serial
import time
import sys
import pynmea2
import rclpy
from rclpy.node import Node, Publisher
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header
from pathlib import Path

DEVICE = '/dev/ttyACM0'
BAUD_RATE = 38400
REACH_LOG = Path.home() / 'reach_log.txt'

class EBP(pynmea2.TalkerSentence):
    """Class to parse EBP messages from Reach RS+ GNSS receivers."""
    
    sentence_type = 'EBP'
    
    fields = (
        ("Latitude", "lat"),
        ("Latitude Direction", "lat_dir"),
        ("Longitude", "lon"),
        ("Longitude Direction", "lon_dir"),
        ("Altitude", "altitude", float),
        ("Altitude Units", "altitude_units")
    )

# Serial device
class SerialDevice:
    def __init__(self, device, baud_rate):
        self.device = device
        self.baud_rate = baud_rate
        self.timeout = 1

# Class to read NMEA data   
class NMEAReader(Node):
    def __init__(self, debug=False):
        """
        Initialize the NMEAReader node.
        Opens 2 publishers for GNSS fix and base fix (not really a fix but uses fix object), 
        and sets up serial communication with the specified device.
        """

        super().__init__('nmea_reader')

        self.debug = debug
        self.log('w', f"[WARN] Debug is enabled, Reach output is saved to {REACH_LOG}") if self.debug else self.log('w', "[WARN] Debug is disabled")

        self.publisher = self.create_publisher(NavSatFix, 'gnss_fix', 10)  
        self.base_publisher = self.create_publisher(NavSatFix, 'base_fix', 10)    

        # logging Reach output for debugging
        self.debug_file = None
        if self.debug:
            self.debug_file = open(REACH_LOG, 'a+')
            self.debug_file.write(f"--- STARTING REACH LOG @ {time.strftime('(%d/%m %H:%M:%S) ---', time.localtime())}\n")

        # Serial device setup
        serialdevice = SerialDevice(DEVICE, BAUD_RATE)
        try:
            self.openSerial(serialdevice)        
            self.readSerial()
        except serial.SerialException as e:
            self.log('e', f"[NMEA/Serial] Couldn't open serial: {e}")
            if self.debug_file:
                self.debug_file.close()
            sys.exit(1)

    # log function to combine log and print into one cuz clean code
    # types:
    #   e - error
    #   i - info
    #   w - warn
    #   d - debug
    def log(self, type, msg):
        if type == 'e':
            print("\033[31m", self.get_logger().error(msg), "\033[0m")
        elif type == 'i':
            self.get_logger().info(msg)
        elif type == 'w':
            print("\033[35m", self.get_logger().warn(msg), "\033[0m")
        elif type == 'd':
            print("\033[36m", f"[DEBUG] {repr(msg)}", "\033[0m")
        else:
            print("[NMEA/Logger]: Unknown msg type, printing as warn")
            print("\033[35m", self.get_logger().warn(msg), "\033[0m")
            

    # Take the serial device and try to open the port
    def openSerial(self, SerialDevice: SerialDevice):
        self.ser = serial.Serial(SerialDevice.device, SerialDevice.baud_rate, timeout=SerialDevice.timeout)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        return None
    
    # Continuously read the serial port and parse NMEA data
    def readSerial(self):
        scanning = True

        if self.ser.is_open:
            self.log('i', "[NMEA/Serial] Serial open, trying read")

            while scanning and rclpy.ok():
                line = self.ser.readline().decode('ascii', errors='replace')
                if len(line) < 7:
                    self.log('w', "[NMEA/Reader] Received an incomplete message, skipping")
                    continue
                
                # write Reach output to log for debugging
                if self.debug_file:
                    self.debug_file.write(line)
                    self.debug_file.flush()

                # read the first 6 characters to determine the type of NMEA message
                if line[3:6] == 'GGA' or line[3:6] == 'EBP':
                    self.log('d', line)
                    try:
                        natsavfix = self.parseNMEA(line)
                        if type(natsavfix) is not NavSatFix:
                            self.log('w', "[NMEA/Reader] Couldn't parse message!")
                        elif line[3:6] == 'GGA':
                            self.publishPosition(natsavfix, self.publisher)
                        else:
                            self.publishPosition(natsavfix, self.base_publisher)
                    except pynmea2.ParseError as e:
                        self.log('e', f"[NMEA/Reader] ERROR: PyNMEA error: {e}")                    

        else:
            self.log('e', "[NMEA/Reader] Serial port is closed, cannot read data")
    
    # Parse NMEA data
    def parseNMEA(self, line):
        msg = pynmea2.parse(line)
        if not isinstance(msg, pynmea2.types.talker.GGA) and not isinstance(msg, EBP):
            self.log('e', "[NMEA/Parser] ERROR: Received a non-GGA/EBP message, this shouldn't happen")
            return None
        else:
            navsat_fix = self.msgToNavSatFix(msg)
            
            return navsat_fix
        
    def msgToNavSatFix(self, msg: pynmea2.types.talker.GGA | EBP) -> NavSatFix:
        navsat_fix = NavSatFix()
        if msg.lat == '' or msg.lon == '':
            return navsat_fix
        
        # pynmea doesnt apply the sign
        lat_sign = -1.0 if msg.lat_dir == 'S' else 1.0
        lon_sign = -1.0 if msg.lon_dir == 'W' else 1.0
        
        navsat_fix.latitude = float(msg.lat) * lat_sign
        navsat_fix.longitude = float(msg.lon) * lon_sign

        navsat_fix.altitude = float(msg.altitude) if isinstance(msg.altitude, (int, float)) else 0.0

        # EBP base station only requires signed coordinates and alt
        if isinstance(msg, EBP):
            navsat_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            return navsat_fix
        
        navsat_fix.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id="gnss"
        )
        
        # Status mapping from NMEA quality indicator to ROS 2 status
        # 0 = No fix, 1 = GPS fix, 2 = DGPS fix, 4 = RTK fixed, 5 = RTK float
        ros_status = NavSatStatus.STATUS_NO_FIX
        if msg.gps_qual in [1, 2]:
            ros_status = NavSatStatus.STATUS_FIX
        elif msg.gps_qual in [4, 5]:
            ros_status = NavSatStatus.STATUS_GBAS_FIX

        navsat_fix.status = NavSatStatus(
            status=ros_status, 
            service=NavSatStatus.SERVICE_GPS
        )
        
        # Covariance estimation from HDOP
        dil = getattr(msg, 'horizontal_dil', None)
        if dil is not None:
            base_accuracy = 2.0  # Assumed standard device accuracy factor
            variance = (float(dil) * base_accuracy) ** 2
            navsat_fix.position_covariance = [
                variance, 0.0, 0.0,
                0.0, variance, 0.0,
                0.0, 0.0, (variance * 4.0) # Vertical error is approximated higher
            ]
            navsat_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        else:
            navsat_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        return navsat_fix

    def publishPosition(self, navfixobj, publisher: Publisher):
        if type(navfixobj) is not NavSatFix:
            self.log('w', "[NMEA/Publisher] WARN: Received a non-NavSatFix message, this shouldn't happen")
        else:
            self.log('i', time.strftime("[NMEA] (%H:%M:%S) ", time.localtime()) + f"Lat: {navfixobj.latitude} Lon: {navfixobj.longitude}" + (f" Alt: {navfixobj.altitude}"))
            publisher.publish(navfixobj)
        return None

    def closeSerial(self, ser=None):
        if ser is None:
            ser = self.ser
        if not ser.is_open:
            self.log('w', "[NMEA/Serial] Serial port is already closed")
        else:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            ser.close()
            if ser.is_open:
                self.log('w', "[NMEA] Serial port is still open for some reason")
            else:
                self.log('i', "[NMEA] Serial port closed successfully")
            del ser        
        sys.exit(0) 

    def __del__(self):
        if self.debug_file:
            self.debug_file.close()
        self.closeSerial()

def main():
    rclpy.init()
    reader = NMEAReader(debug=True)
    rclpy.spin(reader)
    reader.closeSerial()
    rclpy.shutdown()
    sys.exit(0)

if __name__ == "__main__":
    print("NMEA Reader was not started as a ROS node, aborting...")
    sys.exit(1)