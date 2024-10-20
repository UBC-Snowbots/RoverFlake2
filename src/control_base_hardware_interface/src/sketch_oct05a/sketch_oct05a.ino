const int potPins[6] = {A0, A1, A2, A3, A4, A5};

float minRange[6] = {-1.5, -1.9, -1.5, -1.5, -2.0, -10.0};
float maxRange[6] = {1.5, 1.0, 1.5, 1.5, 1.050, 10.0};

float potValues[6];

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Read, map, and print the potentiometer values
  for (int i = 0; i < 6; i++) {
    int rawValue = analogRead(potPins[i]); // Read raw value (0-1023)
    potValues[i] = mapToRange(rawValue, minRange[i], maxRange[i]);
//    Serial.print("Pot ");
//    Serial.print(i);
//    Serial.print(",");
    Serial.print(potValues[i]);
    Serial.print(",");
  }
  Serial.println();

  delay(500); 
}

// Function to map raw potentiometer values (0-1023) to a specified range
float mapToRange(int rawValue, float minVal, float maxVal) {
  return minVal + (float(rawValue) / 1023.0) * (maxVal - minVal);
}
