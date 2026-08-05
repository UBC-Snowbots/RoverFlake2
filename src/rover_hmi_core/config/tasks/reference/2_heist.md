# Heist Mission

Conducted **in darkness**. Only **visible-light** illumination allowed — IR detectors are present.

## Sequence
1. Avoid **tripwires**; evade moving cameras (spotlit fields of view).
2. Access panel: **cut one of the wires** to disable cameras.
3. Security console: decode **morse flashing on the LED** (T = 66.7 ms / 15 Hz — rover_vision pipeline is tuned to this).
4. **Log in with the console password** via XLR/serial terminal.
5. Enter the **vault code with the morse key** (use HMI encode panel).
6. Carry artifact to the extraction point.

## Morse pipeline (rover_vision)
- `morse_camera_pub_node` → `cam_0/morse_led_brightness` (Int32, needs 30 FPS, MJPG forced, threshold 220)
- `morse_decoder_node` → `/morse_decoded` (String, grows per letter, resets per word)
- HMI Morse panel shows LED state, decoded text, manual decode, and encode.
