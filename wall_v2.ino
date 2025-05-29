// IR sensor pins
const int numIRSensors = 2; 
const int IRPins[numIRSensors] = {A2, A3};  // A2 = Left, A3 = Right
unsigned int IRValues[numIRSensors];
float voltageIR[numIRSensors];
float distanceIR[numIRSensors];

void setup() {
  Serial.begin(9600);
}

void loop() {
  wall_following();
  delay(1000);  
}

void IR_Sensor_Reading() {
  for (int i = 0; i < numIRSensors; i++) {
    IRValues[i] = analogRead(IRPins[i]);
    voltageIR[i] = IRValues[i] * (5.0 / 1023.0);
    distanceIR[i] = (-13.0 * voltageIR[i]) + 23.2;
    if (distanceIR[i] < 2) distanceIR[i] = 2;
    if (distanceIR[i] > 15) distanceIR[i] = 15;
  }
}

void wall_following() {
  IR_Sensor_Reading();

  float leftDistance = distanceIR[0];   // A2 = Left
  float rightDistance = distanceIR[1];  // A3 = Right

  const float idealWallDistance = 10.0;     // ideal distance from wall 
  const float margin = 1.5;                 // error margin

  // left wall check
  if (leftDistance >= (idealWallDistance - margin) && leftDistance <= (idealWallDistance + margin)) {
    Serial.println("Left: OK (within ideal distance)");
  } else if (leftDistance < (idealWallDistance - margin)) {
    Serial.println("Left: TOO CLOSE - steer right");
  } else {
    Serial.println("Left: TOO FAR - steer left");
  }

  // right wall check
  if (rightDistance >= (idealWallDistance - margin) && rightDistance <= (idealWallDistance + margin)) {
    Serial.println("Right: OK (within ideal distance)");
  } else if (rightDistance < (idealWallDistance - margin)) {
    Serial.println("Right: TOO CLOSE - steer left");
  } else {
    Serial.println("Right: TOO FAR - steer right");
  }

  Serial.print("Left: "); Serial.print(leftDistance, 2);
  Serial.print(" cm\tRight: "); Serial.print(rightDistance, 2);
  Serial.println(" cm\n");
}
