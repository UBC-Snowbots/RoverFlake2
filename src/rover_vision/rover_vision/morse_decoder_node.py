from enum import Enum
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

MORSE_CODE = {
    '.-': 'a', '-...': 'b', '-.-.': 'c', '-..': 'd', '.': 'e', 
    '..-.': 'f', '--.': 'g', '....': 'h', '..': 'i', '.---': 'j', 
    '-.-': 'k', '.-..': 'l', '--': 'm', '-.': 'n', '---': 'o', 
    '.--.': 'p', '--.-': 'q', '.-.': 'r', '...': 's', '-': 't', 
    '..-': 'u', '...-': 'v', '.--': 'w', '-..-': 'x', '-.--': 'y', 
    '--..': 'z', '-----': '0', '.----': '1', '..---': '2', '...--': '3', 
    '....-': '4', '.....': '5', '-....': '6', '--...': '7', '---..': '8', 
    '----.': '9', '.-.-.-': '.', '--..--': ',', '---...': ':', '..--..': '?',
    '.----.': "'", '-....-': '-', '-..-.': '/', '-.--.': '(', '-.--.-': ')',
    '.-..-.': '"', '.--.-.': '@', '-...-': '='
}

# T-value defined in the CIRC task specs is 66.7ms (15 Hz)
# At 30 frames per second, this is 2 frames
T_FRAMES = 2

# Morse code constants
MORSE_DOT = 1 * T_FRAMES
MORSE_DASH = 3 * T_FRAMES
MORSE_SYMBOL_SPACE = 1 * T_FRAMES
MORSE_LETTER_SPACE = 3 * T_FRAMES
MORSE_WORD_SPACE = 7 * T_FRAMES

# Tolerance thresholds to recover messages even if a frame is missed
# This tolerance can be lowered if the camera has a faster frame rate, but for 30FPS it is the midpoint between each value
MORSE_LETTER_DASH_TOLERANCE = 1 * T_FRAMES
MORSE_WORD_TOLERANCE = 2 * T_FRAMES

# Threshold between dots and dashes
MORSE_DASH_THRESHOLD = (MORSE_DOT + MORSE_DASH) // 2

# Threshold between spaces between symbols and spaces between letters
MORSE_LETTER_SPACE_THRESHOLD = (MORSE_SYMBOL_SPACE + MORSE_LETTER_SPACE) // 2

# Threshold between spaces between letters and spaces between words
MORSE_WORD_SPACE_THRESHOLD = (MORSE_LETTER_SPACE + MORSE_WORD_SPACE) // 2

BRIGHTNESS_THRESHOLD = 220

class LED_State(Enum):
    ON = 0
    OFF = 1

class MorseDecoderNode(Node):
    def __init__(self):
        super().__init__("morse_decoder")

        self.camera_sub = self.create_subscription(
            Int32,
            "cam_0/morse_led_brightness",
            self.signal_callback,
            10
        )
        self.msg_pub = self.create_publisher(String, "/morse_decoded", 10)

        self.last_state = LED_State.OFF
        self.frame_count = 0
        self.morse_msg = ""
        self.msg = ""

        self.get_logger().info("Morse Decoder Node initialized")

    def signal_callback(self, msg):
        state = LED_State.ON if msg.data > BRIGHTNESS_THRESHOLD else LED_State.OFF
        if state == self.last_state:
            self.frame_count += 1

            # Extra checks to make sure the last letter of a word doesn't hang in the buffer until the next word starts
            if state == LED_State.OFF and self.frame_count == MORSE_LETTER_SPACE_THRESHOLD and self.morse_msg:
                self.process_char(LED_State.OFF, self.frame_count)
        else:
            self.process_char(self.last_state, self.frame_count)
            self.last_state = state
            self.frame_count = 1

    def process_char(self, state, duration):
        if state == LED_State.ON:
            if duration < MORSE_DASH_THRESHOLD:
                self.morse_msg += "."
            else:
                self.morse_msg += "-"
        else:
            if duration >= MORSE_LETTER_SPACE_THRESHOLD:
                # Space is between letters within a word, append the letter to the existing message
                if self.morse_msg:
                    letter = MORSE_CODE.get(self.morse_msg, "?")
                    self.msg += letter

                    msg = String()
                    msg.data = self.msg
                    self.msg_pub.publish(msg) # Publish full message each time a new letter is added
                    
                    self.morse_msg = ""

            if duration >= MORSE_WORD_SPACE_THRESHOLD:
                # End of a word, reset the alphabetic message
                self.msg = ""

def main(args=None):
    rclpy.init(args=args)
    node = MorseDecoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
