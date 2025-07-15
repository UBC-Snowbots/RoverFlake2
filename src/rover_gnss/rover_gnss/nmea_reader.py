import serial
import time
import sys
import pynmea2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import math

DEVICE = '/dev/ttyACM0'
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
        self.angle_publisher = self.create_publisher(String, 'antenna_angle', 10)

        # Dummy CS coordinates (comms station)
        self.cs_lat = 43.6532
        self.cs_lon = -79.3832

        # Serial part
        serialdevice = SerialDevice(DEVICE, BAUD_RATE)
        self.openSerial(serialdevice)        
        self.timer = self.create_timer(0.5, self.readSerial)

    def openSerial(self, SerialDevice):
        try:
            self.ser = serial.Serial(SerialDevice.device, SerialDevice.baud_rate, timeout=SerialDevice.timeout)
            self.ser.flushInput()
            self.ser.flushOutput()
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except FileNotFoundError:
            print(f"[NMEA/Serial] !!! Serial port not found: {DEVICE}\nTerminating...")
            sys.exit(1)

    def readSerial(self):
        scanning = True
        while scanning and rclpy.ok():
            try:
                if self.ser.isOpen():
                    line = self.ser.readline().decode('ascii', errors='replace')                
                    if line.startswith('$GP'):
                        print(line)
                        if line.startswith('$GNEBP'):
                            break
                        try:
                            natsavfix = self.parseNMEA(line)
                            if isinstance(natsavfix, NavSatFix):
                                self.publishPosition(natsavfix)

                                # Compute bearing angle
                                rover_lat = natsavfix.latitude
                                rover_lon = natsavfix.longitude
                                bearing = self.compute_bearing(self.cs_lat, self.cs_lon, rover_lat, rover_lon)

                                print(f"[ANTENNA] Rotate antenna to {bearing:.2f}°")

                                # Publish bearing angle
                                angle_msg = String()
                                angle_msg.data = f"{bearing:.2f}"
                                self.angle_publisher.publish(angle_msg)
                            else:
                                print("[NMEA/Reader] Non-NSF message, skipping...")
                        except pynmea2.ParseError:
                            pass
                else:
                    print(time.strftime("[NMEA/DEBUG] (%H:%M:%S) ", time.localtime()) + "CLOSED")
            except KeyboardInterrupt:
                break

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
            print(f"[NMEA/Parser] !!! Non-GGA message: {msg}")
            return None

    def publishPosition(self, navfixobj):
        if not isinstance(navfixobj, NavSatFix):
            print("[NMEA/Publisher] !!! Received a non-NavSatFix message")
            return
        self.get_logger().info('Publishing NMEA data...')
        print(time.strftime("[NMEA] (%H:%M:%S) ", time.localtime()) + 
              f"Lat: {navfixobj.latitude}, Lon: {navfixobj.longitude}, Alt: {navfixobj.altitude}")
        self.publisher.publish(navfixobj)

    def compute_bearing(self, lat1, lon1, lat2, lon2):
        """
        Compute initial bearing from (lat1, lon1) to (lat2, lon2)
        Returns bearing in degrees from North.
        """
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_lambda = math.radians(lon2 - lon1)

        x = math.sin(delta_lambda) * math.cos(phi2)
        y = math.cos(phi1) * math.sin(phi2) - \
            math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)

        bearing = math.atan2(x, y)
        bearing = math.degrees(bearing)
        return (bearing + 360) % 360  # Normalize

    def closeSerial(self, ser=None):
        if ser is None:
            ser = self.ser
        if ser.isOpen():
            ser.flushInput()
            ser.flushOutput()
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            ser.close()
            if not ser.isOpen():
                print("[NMEA] Serial port closed successfully")
        else:
            print("[M+] !! Serial port is already closed")
        sys.exit(0)

def main():
    rclpy.init()
    reader = NMEAReader()
    rclpy.spin(reader)
    reader.closeSerial()
    rclpy.shutdown()
    sys.exit(0)