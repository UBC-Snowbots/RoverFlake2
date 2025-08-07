#include <Servo.h>

// Constants
double Pos2 = 0.0; // tilt

Servo servo1;
Servo servo2;

// === Constants for gas sensor ===
const float RL = 10000.0;         // 10kΩ
const float Ro = 5100000.0;       // Your calibrated Ro
const float Vcc = 5.0;
const float adcMax = 1023.0;

// === Datasheet fit: H2Curve = {log(ppm), log(rs/ro), slope} ===
float H2Curve[3] = {2.3, 0.93, -1.44};
bool flag = false;

// === Serial Buffer ===
String inputString = "";
bool stringComplete = true;

bool status = false;

void setup() {
  Serial.begin(9600);
  inputString.reserve(50);
  servo1.attach(10);
  servo2.attach(11);
}

void get_sensor_data() {
  if (status) {
    int adcVal = analogRead(A0);
    float Vout = (adcVal / adcMax) * Vcc;

    if (Vout < 0.01) Vout = 0.01; // avoid division by zero

    float Rs = RL * ((Vcc / Vout) - 1.0);
    float ratio = Rs / Ro;

    float logRatio = log10(ratio);
    float ppm_log = (logRatio - H2Curve[1]) / H2Curve[2];
    float ppm = pow(10, ppm_log);

    Serial.print("$H2_reading_PPM(");
    Serial.print(ppm, 2);
    Serial.println(")");
  }
}

void control_ptz(String cmd) {
  if (cmd.length() < 2) return;

  char direction = cmd.charAt(0);
  float value = cmd.substring(1).toFloat();

  if (direction == '0') { // continuous pan
    int speed = 90 + value; // 90 = stop
    speed = constrain(speed, 0, 180);
    servo1.write(speed);

    Serial.print("$pan_speed(");
    Serial.print(speed);
    Serial.println(")");
  } else if (direction == '1') { // tilt (standard servo)
    Pos2 += value;
    Pos2 = constrain(Pos2, 0, 180);
    servo2.write((int)Pos2);
    Serial.print("$tilt(");
    Serial.print(Pos2, 2);
    Serial.println(")");
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
//    Serial.print("I am working");
    if (inChar == '\n') {
      stringComplete = true;
      break;
    } else {
      inputString += inChar;
//      Serial.print(inChar);
//      Serial.print(inputString);
    }
  }
}

void loop() {
  serialEvent();
//  Serial.print(stringComplete);
  if (stringComplete) {
    inputString.trim();
    Serial.print(inputString);
    if (inputString == "sensor_on") {
      status = true;
    } else if (inputString == "sensor_off") {
      status = false;
    } else if (inputString == "stop_pan") {
      servo1.write(90); // stop continuous servo
      Serial.println("$pan_stop");
    } else if (inputString.charAt(0) == '0' || inputString.charAt(0) == '1') {
      control_ptz(inputString);
    } else {
      Serial.println("Invalid command.");
    }

    inputString = "";
    stringComplete = false;
  }
  get_sensor_data();
  delay(100);
}