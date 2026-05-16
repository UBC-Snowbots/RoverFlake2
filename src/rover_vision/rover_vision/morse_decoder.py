import cv2
from cv_bridge import CvBridge
from enum import Enum
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

MORSE_CODE = {
    '.-': 'A', '-...': 'B', '-.-.': 'C', '-..': 'D', '.': 'E', 
    '..-.': 'F', '--.': 'G', '....': 'H', '..': 'I', '.---': 'J', 
    '-.-': 'K', '.-..': 'L', '--': 'M', '-.': 'N', '---': 'O', 
    '.--.': 'P', '--.-': 'Q', '.-.': 'R', '...': 'S', '-': 'T', 
    '..-': 'U', '...-': 'V', '.--': 'W', '-..-': 'X', '-.--': 'Y', 
    '--..': 'Z', '-----': '0', '.----': '1', '..---': '2', '...--': '3', 
    '....-': '4', '.....': '5', '-....': '6', '--...': '7', '---..': '8', 
    '----.': '9'
}

# T-value defined in the CIRC task specs is 66.7ms
# At 30 frames per second, this is 2 frames
T_FRAMES = 2

# Morse code constants
MORSE_DOT = 1
MORSE_DASH = 3
MORSE_LETTER_SPACE = 3
MORSE_WORD_SPACE = 7

# Frame processing constants
PIXEL_BRIGHTNESS_THRESHOLD = 240
PIXEL_MAX_BRIGHTNESS = 255
IMAGE_BRIGHTNESS_THRESHOLD = 5000

class LED_State(Enum):
    ON = 0
    OFF = 1

class MorseDecoderNode(Node):
    def __init__(self):
        super().__init__("morse_decoder")
        self.bridge = CvBridge()

        self.camera_sub = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.image_callback,
            10
        )
        self.msg_pub = self.create_publisher(String, "/morse_decoded", 10)

        self.last_state = LED_State.OFF
        self.frame_count = 0
        self.morse_msg = ""
        self.msg = ""

        self.get_logger().info("Morse Decoder Node initialized")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, threshold = cv2.threshold(grayscale, PIXEL_BRIGHTNESS_THRESHOLD, PIXEL_MAX_BRIGHTNESS, cv2.THRESH_BINARY)

            state = LED_State.ON if np.sum(threshold) > IMAGE_BRIGHTNESS_THRESHOLD else LED_State.OFF
            if state == self.last_state:
                self.frame_count += 1
            else:
                self.process_char(self.last_state, self.frame_count)
                self.last_state = state
                self.frame_count = 1

        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")

    def process_char(self, state, duration):
        if state == LED_State.ON:
            if duration == MORSE_DOT:
                self.morse_msg += "."
            else:
                self.morse_msg += "-"
        else:
            if duration == MORSE_LETTER_SPACE:
                if self.morse_msg:
                    letter = MORSE_CODE[self.morse_msg]
                    self.msg += letter
                    self.msg_pub.publish(self.msg) # Publish full message each time a new letter is added
            else:
                self.msg_pub.publish(self.msg)
                self.morse_msg = ""
                self.msg = ""

def main(args=None):
    rclpy.init(args=args)
    node = MorseDecoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destoryAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
