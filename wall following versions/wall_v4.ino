const int IR_LEFT  = A2;
const int IR_FRONT = A4;
unsigned int IRValues[2];
float voltageIR[2];
float distanceIR[2];

// Wall following parameters
const float idealWallDistance = 10.0;     // cm, ideal distance from left wall
const float frontThreshold    = 6.0;      // cm, threshold for detecting wall ahead
const float Kp_wall          = 5.0;       // proportional gain for wall following

const int BASE_SPEED = 40;
const int MAX_SPEED  = 70;
const int MIN_SPEED  = 20;

enum RobotState { WALL_FOLLOWING, TURNING };
RobotState state = WALL_FOLLOWING;
unsigned long turnStartTime = 0;
const unsigned long TURN_DURATION = 1000; // ms for a 90-degree turn, adjust as needed

void setup() {
  Serial.begin(9600);
}

void loop() {
  switch (state) {
    case TURNING:
      if (millis() - turnStartTime >= TURN_DURATION) {
        stopMotors();
        state = WALL_FOLLOWING;
        Serial.println("Turn complete - resuming wall following");
      } else {
        turnRight();
      }
      break;

    case WALL_FOLLOWING:
      wall_following_logic();
      break;
  }
  delay(50);
}

void IR_Sensor_Reading() {
  int pins[2] = {IR_LEFT, IR_FRONT};
  for (int i = 0; i < 2; i++) {
    IRValues[i] = analogRead(pins[i]);
    voltageIR[i] = IRValues[i] * (5.0 / 4095.0);
    distanceIR[i] = (-13.0 * voltageIR[i]) + 23.2;
    if (distanceIR[i] < 2.0) distanceIR[i] = 2.0;
    if (distanceIR[i] > 15.0) distanceIR[i] = 15.0;
  }
}

void wall_following_logic() {
  IR_Sensor_Reading();

  float leftDistance  = distanceIR[0];
  float frontDistance = distanceIR[1];

  bool frontWall = frontDistance < frontThreshold;
  bool leftWall  = leftDistance < 14.0; // If sensor sees something within 14cm, it's a wall

  Serial.print("Left: "); Serial.print(leftDistance, 1);
  Serial.print(" cm\tFront: "); Serial.print(frontDistance, 1);
  Serial.println(" cm");

  // If wall in front and on the left, turn right 90 deg
  if (frontWall && leftWall) {
    Serial.println("Front and left wall detected - turn RIGHT by 90 deg");
    state = TURNING;
    turnStartTime = millis();
    turnRight();
    return;
  }

  // Otherwise, follow the left wall at a constant distance
  float error = idealWallDistance - leftDistance; // Positive if too far from wall

  int correction = Kp_wall * error;
  int leftSpeed  = constrain(BASE_SPEED + correction, MIN_SPEED, MAX_SPEED);
  int rightSpeed = constrain(BASE_SPEED - correction, MIN_SPEED, MAX_SPEED);

  setMotorSpeeds(leftSpeed, rightSpeed);

  Serial.print("correction: "); Serial.print(correction);
  Serial.print("\tLeft Speed: "); Serial.print(leftSpeed);
  Serial.print("\tRight Speed: "); Serial.println(rightSpeed);
}

// --- Motor control (fill these for your hardware) ---

void turnRight() {
  // Example: left motor forward, right motor backward
  Serial.println("Turning right.");
  // analogWrite(leftMotorPin, TURN_SPEED);
  // analogWrite(rightMotorPin, 0);
}

void stopMotors() {
  Serial.println("Motors stopped.");
  // analogWrite(leftMotorPin, 0);
  // analogWrite(rightMotorPin, 0);
}

void setMotorSpeeds(int left, int right) {
  // Map left/right to your motor driver here
  // analogWrite(leftMotorPin, left);
  // analogWrite(rightMotorPin, right);
}
