#include <Servo.h>

// Constants
int Pos2 = 90; // tilt mid pos
int speed;

//Servo servo1;
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
bool gstat = false;

void setup() {
  Serial.begin(9600);
  inputString.reserve(50);
//  servo1.attach(10);
  pinMode(3,OUTPUT);
  pinMode(5,OUTPUT);
  digitalWrite(3,LOW);
  digitalWrite(5,LOW);
  delay(10);
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

void geigar(){
  if (gstat){
    int mSv = analogRead(A1)
    Serial.print("$GEI(");
    Serial.print(mSv, 2);
    Serial.println(")");
  }

}

void control_ptz(String cmd) {
  if (cmd.length() < 2) return;

  char direction = cmd.charAt(0);
  float value = cmd.substring(1).toFloat();
//  speed = value;
//  speed = constrain(speed, 0, 255);
//  Serial.print(speed);

  if (direction == '0') { // continuous pan
    char panDir = cmd.charAt(1);
    Serial.print(panDir);
    if (panDir == '+'){
      // positive value
      digitalWrite(5, LOW);
//      analogWrite(3, speed);
      digitalWrite(3, HIGH);  
      delay(50);
      digitalWrite(3, LOW);
    }
    else{
//    speed = contrain(speed, 0, 255);
      digitalWrite(3, LOW);
//      analogWrite(5,speed);
      digitalWrite(5, HIGH);
      delay(50);
      digitalWrite(5, LOW);
      
      delay(100);
    }
    Serial.print(speed);
    Serial.print(direction);
    
  } else if (direction == '1') { // tilt (positional servo)
    Pos2 += value;
    Pos2 = constrain(Pos2, 80, 165);
    Serial.print(Pos2, 2); // meow
    servo2.write(Pos2);
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
      break;
    } else {
      inputString += inChar;
    }
  }
}

void loop() {
  if (stringComplete) {
    inputString.trim();
    Serial.print(inputString);
    if (inputString == "sensor_on") {
      status = true;
    } 
    else if (inputString == "sensor_off") {
      status = false;
    } 
    else if (inputString == "geigar_on"){
      gstat = true;
    }
    else if (inputString == "geigar_off"){
      gstat = false;
    }
    else if (inputString == "stop_pan") {
      digitalWrite(3, LOW);
      digitalWrite(5, LOW);
    } 
    else if (inputString.charAt(0) == '0' || inputString.charAt(0) == '1') {
      control_ptz(inputString);
    } 
    else {
      Serial.println("Invalid command.");
    }
    inputString = "";
    stringComplete = false;
  }
  get_sensor_data();
  delay(100);
}