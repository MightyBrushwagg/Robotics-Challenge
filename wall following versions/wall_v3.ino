const int IR_LEFT  = A2;
const int IR_RIGHT = A3;
const int IR_FRONT = A4;
unsigned int IRValues[3];
float voltageIR[3];
float distanceIR[3];

//wall following constants
const float idealWallDistance = 10.0;
const float margin = 1.5;
const float frontThreshold = 6.0;
const float Kp_wall = 5.0;

const int BASE_SPEED = 40;
const int MAX_SPEED = 70;
const int MIN_SPEED = 20;

//multiple states of wall following
enum RobotState { WALL_FOLLOWING, TURNING };
RobotState state = WALL_FOLLOWING;
unsigned long turnStartTime = 0;
const unsigned long TURN_DURATION = 1000; // milliseconds for 90-degree turn; adjust as needed

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
        turnRight(); // if action is not complete, carry on turning
      }
      break;

    case WALL_FOLLOWING:
      wall_following_logic();
      break;
  }
  delay(50); 
}

void IR_Sensor_Reading() {
  int pins[3] = {IR_LEFT, IR_RIGHT, IR_FRONT};
  for (int i = 0; i < 3; i++) {
    IRValues[i] = analogRead(pins[i]);
    voltageIR[i] = IRValues[i] * (5.0 / 4095.0);
    distanceIR[i] = (-13.0 * voltageIR[i]) + 23.2;
    if (distanceIR[i] < 2.0) distanceIR[i] = 2.0;
    if (distanceIR[i] > 15.0) distanceIR[i] = 15.0;
  }
}

void wall_following_logic() {
  IR_Sensor_Reading();

//gets sensor readings and assigns them to variables
  float leftDistance  = distanceIR[0];
  float rightDistance = distanceIR[1];
  float frontDistance = distanceIR[2];

//assigns condtions based on walls to variables
  bool frontWall = frontDistance < frontThreshold;
  bool rightWall = rightDistance < 14.0;
  bool leftWall  = leftDistance < 14.0;

  Serial.print("Left: "); Serial.print(leftDistance, 1);
  Serial.print(" cm\tRight: "); Serial.print(rightDistance, 1);
  Serial.print(" cm\tFront: "); Serial.print(frontDistance, 1);
  Serial.println(" cm");

  // state change
  if (frontWall && leftWall && !rightWall) {
    Serial.println("front and left wall detected - turn RIGHT by 90 deg");
    state = TURNING;
    turnStartTime = millis();
    turnRight(); // start turning
    return;
  }

  float error;
  if (leftWall && rightWall) {
    Serial.println("Walls on both sides - centering");
    error = rightDistance - leftDistance;
  } else {
    Serial.println("Left wall only - follow left wall");
    error = idealWallDistance - leftDistance;
  }

//centering using differential drive
  int correction = Kp_wall * error;
  int leftSpeed  = constrain(BASE_SPEED + correction, MIN_SPEED, MAX_SPEED);
  int rightSpeed = constrain(BASE_SPEED - correction, MIN_SPEED, MAX_SPEED);  

  setMotorSpeeds(leftSpeed, rightSpeed);

  Serial.print("correction: "); Serial.print(correction);
  Serial.print("\tLeft Speed: "); Serial.print(leftSpeed);
  Serial.print("\tRight Speed: "); Serial.println(rightSpeed);
}


void turnRight() {
  // turn right using servo code
  Serial.println("Turning right.");
}
void stopMotors() {
  // Stop both motors
  Serial.println("Motors stopped.");
}
void setMotorSpeeds(int left, int right) {
  //sets relevant speeds for each motor
}
