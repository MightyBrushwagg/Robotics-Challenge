//0602 testing code
#include <Servo.h>
#include <Motoron.h>

//******************************************************************DEFINITIONS******************************************************************
// Motoron shields on I2C addresses 0x10 (front) and 0x11 (back)
MotoronI2C mc1(0x10), mc2(0x11);

// important control
// int mode = 1; // 0 = dead, 1 = line following, 2 = wall following, 
// int lastmode = 0;
enum RobotMode {
  MODE_DEAD = 0,
  MODE_LINE_FOLLOWING = 1,
  MODE_CROSSROAD_TAKING = 2,
  MODE_WALL_FOLLOWING = 3
};
RobotMode mode = MODE_LINE_FOLLOWING;
RobotMode lastmode = MODE_DEAD;
RobotMode lastPrintedMode = MODE_DEAD;

enum Direction {
  LEFT,
  RIGHT,
  STRAIGHT
};

// Reflectance sensors (13 channels)
const int numReflectanceSensors = 13;
const int reflectorsMiddle = (numReflectanceSensors+1)/2;
// const int sensorPins[numReflectanceSensors] = {31,30,33,32,35,34,37,36,39,41,40,43,42};
const int sensorPins[numReflectanceSensors] = {38,41,31,30,33,32,35,34,37,36,39,40,42};
unsigned int sensorValuesFull[numReflectanceSensors];
unsigned int sensorValuesBinary[numReflectanceSensors];
// bool sensor[numReflectanceSensors];
const unsigned int reflectanceThreshold = 500; 

// Steering servo
#define SERVO_PIN   27
#define FRONT       90
#define SHARP_RIGHT (FRONT + 65)
#define SHARP_LEFT  (FRONT - 65)

// Speed params
#define BASE_SPEED  40
#define MAX_SPEED   70
#define MIN_SPEED   20

// PID gains
//const float Kp = 0, Ki = 0, Kd = 0; 
const float Kp = 30, Ki = 1, Kd = 1; // 30,1,1 best
float integral = 0, previousErr = 0;
const int weight[numReflectanceSensors] = {-6,-5,-4,-3,-2,-1,0,1,2,3,4,5,6};
//const float weight[numReflectanceSensors] = {-27,-16,-9,-4,-1,-0.2,0,0.2,1,4,9,16,27}; //Exponential 

const int IR_LEFT  = A2;
const int IR_RIGHT = A3;
const int IR_FRONT = A4;
const int numIRSensors = 3;
const int IRPins[numIRSensors] = {IR_LEFT, IR_RIGHT, IR_FRONT};
unsigned int IRValues[numIRSensors];
float voltageIR[numIRSensors];
float distanceIR[numIRSensors];

enum RobotStateWall { WALL_FOLLOWING, TURNING };
RobotStateWall stateWall = WALL_FOLLOWING;
unsigned long turnStartTime = 0;
const unsigned long TURN_DURATION = 1000; // milliseconds for 90-degree turn; adjust as needed

//wall following constants
const float idealWallDistance = 10.0;
const float margin = 1.5;
const float frontThreshold = 6.0;
const float Kp_wall = 5.0;

Servo head;

// Crossroads
const int numCrossroads = 3;
const Direction crossroadDirections[numCrossroads] = {RIGHT, LEFT, RIGHT};  // Hard 
int currentCrossroad = 0;
//int crossroading = 0;
unsigned int startedTurn;
const int crossroadThreshold = 5000000;

//Wall Following
int buttonState = HIGH;         // Current debounced state
int lastButtonState = HIGH;     // Previous raw reading

unsigned long lastDebounceTime = 0;  // Last time the input changed
unsigned long debounceDelay = 50;    // Debounce delay
unsigned long cooldownTime = 300;    // Optional cooldown after press
unsigned long lastActionTime = 0;    // Last time action was taken
const int buttonPin = 57;

//Debug Printing
String modeToString(RobotMode m) {
  switch (m) {
    case MODE_DEAD: return "DEAD";
    case MODE_LINE_FOLLOWING: return "LINE FOLLOWING";
    case MODE_CROSSROAD_TAKING: return "CROSSROAD TAKING";
    case MODE_WALL_FOLLOWING: return "WALL FOLLOWING";
    default: return "UNKNOWN";
  }
}

String directionToString(Direction d) {
  switch (d) {
    case LEFT: return "LEFT";
    case RIGHT: return "RIGHT";
    case STRAIGHT: return "STRAIGHT";
    default: return "UNKNOWN";
  }
}

// Functions
void InitializeMotorShield();
void Reflectance_Sensor_Reading();
void IR_Sensor_Reading();
void go_Advance_2_wheel(int leftSpeed, int rightSpeed);
void go_Advance_4_wheel(int leftFrontSpeed, int rightFrontSpeed, int leftBackSpeed, int rightBackSpeed);

void stop_Stop();
float pid(float error);
void take_crossroad();
void check_crossroad();
RobotMode determine_line_status();
void auto_tracking();

void wall_following();
void turnRight();

void button_check();
void wifi_check();

//******************************************************************BODY******************************************************************
void setup() {
  InitializeMotorShield();
  head.attach(SERVO_PIN);
  head.write(FRONT);
  Serial.begin(9600);
  analogReadResolution(12);
}

void loop() {
  Reflectance_Sensor_Reading();

  if (mode != lastPrintedMode) {
    Serial.print("MODE CHANGED: ");
    Serial.println(modeToString(mode));
    lastPrintedMode = mode;
  }

  switch (mode) {
    case MODE_LINE_FOLLOWING: {
      mode = determine_line_status();
      if (mode == MODE_LINE_FOLLOWING) {
        auto_tracking();
      }
      break;
    }

    case MODE_CROSSROAD_TAKING:
      take_crossroad();
      break;

    case MODE_WALL_FOLLOWING:
      wall_following();
      break;

    case MODE_DEAD:
    default:
      stop_Stop();
      break;
  }
}


//******************************************************************FUNCTIONS******************************************************************
void InitializeMotorShield() {
  Wire1.begin();
  mc1.setBus(&Wire1);
  mc2.setBus(&Wire1);

  // First shield (channels 1-2)
  mc1.reinitialize(); mc1.disableCrc(); mc1.clearResetFlag(); mc1.disableCommandTimeout();
  for (uint8_t i = 1; i <= 2; i++) {
    mc1.setMaxAcceleration(i,100);
    mc1.setMaxDeceleration(i,100);
  }
  // Second shield (if used)
  mc2.reinitialize(); mc2.disableCrc(); mc2.clearResetFlag(); mc2.disableCommandTimeout();
  for (uint8_t i = 1; i <= 2; i++) {
    mc2.setMaxAcceleration(i,100);
    mc2.setMaxDeceleration(i,100);
  }
}

void Reflectance_Sensor_Reading() {
  // 1) Charge cycle
  for (int i = 0; i < numReflectanceSensors; i++) {
    pinMode(sensorPins[i], OUTPUT);
    digitalWrite(sensorPins[i], HIGH);
  }
  delayMicroseconds(10);

  // 2) Begin discharge
  unsigned long startTime = micros();
  for (int i = 0; i < numReflectanceSensors; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // 3) Measure discharge time
  bool done[numReflectanceSensors] = {false};
  unsigned long timeout = 3000;
  while (micros() - startTime < timeout) {
    for (int i = 0; i < numReflectanceSensors; i++) {
      if (!done[i] && digitalRead(sensorPins[i]) == LOW) {
        sensorValuesFull[i] = micros() - startTime;
        done[i] = true;
      }
    }
  }

  for (int i = 0; i < numReflectanceSensors; i++) {
    if (!done[i]) sensorValuesFull[i] = timeout;
  }

  //Convert to binary states based on threshold
  for (int i = 0; i < numReflectanceSensors; i++) {
    sensorValuesBinary[i] = (sensorValuesFull[i] < reflectanceThreshold); // i changed the sign
  }

  // // Debug: print raw values and binary state
  // for (int i = 0; i < numReflectanceSensors; i++) {
  //   // Serial.print(sensorValuesFull[i]); Serial.print('\t');
  // }
  // // Serial.print("| ");
  // for (int i = 0; i < numReflectanceSensors; i++) {
  //   Serial.print(sensorValuesBinary[i] ? '1' : '0');
  // }
  // Serial.println();
}

void IR_Sensor_Reading() {
  for (int i=0; i<numIRSensors; i++){
    IRValues[i] = analogRead(IRPins[i]);

    // Convert ADC value (0–4095) to voltage (0–5V)
    voltageIR[i] = IRValues[i] * (5.0 / 4095.0);

    // Approximate distance from voltage based on datasheet
    // From Sharp's graph, at 0.4V = 15cm, at 1.65V = 2cm
    // Reverse linear interpolation between those two:
    distanceIR[i] = (-13.0 * voltageIR[i]) + 23.2;

    // Clamp output to valid range
    if (distanceIR[i] < 2) distanceIR[i] = 2;
    if (distanceIR[i] > 15) distanceIR[i] = 15;

    // Serial.print("Voltage1: ");
    // Serial.print(voltage1, 2);
    Serial.print("\tDistance1: ");
    Serial.print(distanceIR[i], 2);
    // Serial.println(" cm");
  }
}

void go_Advance_2_wheel(int leftSpeed, int rightSpeed) {
  int16_t mL = constrain(leftSpeed, MIN_SPEED, MAX_SPEED) * 4;
  int16_t mR = constrain(rightSpeed, MIN_SPEED, MAX_SPEED) * 4;
  mc1.setSpeed(2, mL);
  mc1.setSpeed(1, mR);
  mc2.setSpeed(2, mL);
  mc2.setSpeed(1, mR);
}

void go_Advance_4_wheel(int leftFrontSpeed, int rightFrontSpeed, int leftBackSpeed, int rightBackSpeed) {
  int16_t FmL = constrain(leftFrontSpeed, MIN_SPEED, MAX_SPEED) * 4;
  int16_t FmR = constrain(rightFrontSpeed, MIN_SPEED, MAX_SPEED) * 4;
  int16_t BmL = constrain(leftBackSpeed, MIN_SPEED, MAX_SPEED) * 4;
  int16_t BmR = constrain(rightBackSpeed, MIN_SPEED, MAX_SPEED) * 4;
  mc1.setSpeed(2, FmL);
  mc1.setSpeed(1, FmR);
  mc2.setSpeed(2, BmL);
  mc2.setSpeed(1, BmR);
}

void stop_Stop() {
  mc1.setSpeed(1, 0);
  mc1.setSpeed(2, 0);
  mc2.setSpeed(1, 0);
  mc2.setSpeed(2, 0);
}

unsigned long lastMicros = 0;
const float integralMax = 100;
float pid(float error) {
  unsigned long now = micros();
  float dt = lastMicros ? (now - lastMicros) / 1e6 : 0.01;  
  lastMicros = now;

  integral    += error * dt;
  integral = constrain(integral, -integralMax, integralMax);

  float derivative = (error - previousErr) / dt;
  previousErr = error;
  return Kp * error + Ki * integral + Kd * derivative;
}

void take_crossroad() {
  if (currentCrossroad >= numCrossroads) {
    mode = MODE_LINE_FOLLOWING;
    return;
  }

  //Debug
  static int lastCrossroadPrinted = -1;
  if (currentCrossroad != lastCrossroadPrinted) {
    Serial.print("Taking Crossroad #");
    Serial.print(currentCrossroad + 1);
    Serial.print(" → ");
    Serial.println(directionToString(crossroadDirections[currentCrossroad]));
    lastCrossroadPrinted = currentCrossroad;
  }

  // Set Angles
  int turnAngle;
  switch (crossroadDirections[currentCrossroad]) {
    case LEFT:     turnAngle = SHARP_LEFT; break;
    case RIGHT:    turnAngle = SHARP_RIGHT; break;
    case STRAIGHT: turnAngle = FRONT; break;
    default:       turnAngle = FRONT;
  }

  head.write(turnAngle);
  Reflectance_Sensor_Reading();

  if (micros() - startedTurn >= crossroadThreshold) {
    currentCrossroad++;
    mode = MODE_LINE_FOLLOWING;
  }
}

void check_crossroad() {
  Reflectance_Sensor_Reading();

  int zeroBefore = 0;
  int zeroThenOne = 0;
  for (int i = 0; i < numReflectanceSensors - 1; i++) {
    if (sensorValuesBinary[i] == 0 && zeroThenOne == 0) {
      zeroBefore = 1;
    } else if (sensorValuesBinary[i] == 1 && zeroBefore == 1) {
      zeroThenOne = 1;
    } else if (sensorValuesBinary[i] == 0 && zeroThenOne == 1) {
      // Confirm whether it is a crossroad
      mode = MODE_CROSSROAD_TAKING;
      startedTurn = micros();
      break;
    }
  }
}

RobotMode determine_line_status() {
    Reflectance_Sensor_Reading();

    // count the number of black lines
    int blackCount = 0;
    for (int i = 0; i < numReflectanceSensors; i++) {
        if (sensorValuesBinary[i]) blackCount++;
    }

    // Case1: All Black
    if (blackCount == numReflectanceSensors) {
        return MODE_DEAD;
    }

    // Case2: All White
    if (blackCount == 0) {
        // Move forward a little bit
        go_Advance_2_wheel(MIN_SPEED, MIN_SPEED);
        delay(100); 
        Reflectance_Sensor_Reading();

        int blackCountAfterMove = 0;
        for (int i = 0; i < numReflectanceSensors; i++) {
            if (sensorValuesBinary[i]) blackCountAfterMove++;
        }

        if (blackCountAfterMove == 0) {
            return MODE_WALL_FOLLOWING; // Case3: Lose Lines
        } else {
            // Search with Swings
            head.write(110);
            delay(80);
            head.write(70);
            delay(80);
            head.write(FRONT);
            return MODE_LINE_FOLLOWING;
        }
    }

    // Case4: Crossroad
    if (blackCount >= 4) {  // the number could be adjusted
        // such as : black white black white black
        for (int i = 1; i < numReflectanceSensors - 1; i++) {
            if (!sensorValuesBinary[i - 1] && sensorValuesBinary[i] && !sensorValuesBinary[i + 1]) {
                return MODE_CROSSROAD_TAKING;
            }
        }
    }

    // Case5: Normal Line Tracking
    return MODE_LINE_FOLLOWING;
}

// Line-following with front-wheel PID steering
void auto_tracking() {
  Reflectance_Sensor_Reading();
  
  int sumW = 0, sumH = 0;
  for (int i = 0; i < numReflectanceSensors; i++) {
    if (sensorValuesBinary[i]) { 
      sumW += weight[i]; 
      sumH++; 
    } 
  }
  float error = (sumH>0) ? float(sumW)/sumH : 0;

  //************************DBUG*************************
  // // Serial.print("Err: "); Serial.print(error, 2);
  float output = pid(error);  
  // // Serial.print("  Out: "); Serial.print(output, 2);
  int angle = constrain(FRONT - int(output), SHARP_LEFT, SHARP_RIGHT);
  // // Serial.print("  Angle: "); Serial.println(angle);

  // Steering output: map PID output to servo angle
  //int angle = constrain(FRONT - int(output), SHARP_LEFT, SHARP_RIGHT);
  
  // Serial.println(angle);
  //delay(150);
  
  //Compute weighted centroid error using raw values
  
  // Speed output: reduce speed if steering correction is large
  //int speedBase = constrain(BASE_SPEED - abs(int(output)) * 3, MIN_SPEED, MAX_SPEED); 
  float speedDrop = constrain(abs(output) * 0.5, 0, BASE_SPEED - MIN_SPEED);
  int speedBase = BASE_SPEED - speedDrop;
  int leftSpd   = speedBase - int(output * 1.5);
  int rightSpd  = speedBase + int(output * 1.5);
  // Serial.println(leftSpd);
  // Serial.println(rightSpd);

    go_Advance_2_wheel(leftSpd, rightSpd);
    head.write(angle);
  }
}

void wall_following() {
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
    stateWall = TURNING;
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

  go_Advance_2_wheel(leftSpeed, rightSpeed);

  Serial.print("correction: "); Serial.print(correction);
  Serial.print("\tLeft Speed: "); Serial.print(leftSpeed);
  Serial.print("\tRight Speed: "); Serial.println(rightSpeed);
}

void turnRight() {
  // turn right using servo code
  Serial.println("Turning right.");
  head.write(SHARP_RIGHT);
  go_Advance_2_wheel(BASE_SPEED+20, BASE_SPEED-20);
}

//button_check();
//wifi_check();

void button_check() {
  
  int reading = digitalRead(buttonPin);
  unsigned long currentTime = millis();
  // if (buttonState == HIGH) {
  //       Serial.println("Button Pressed!");
  //       //digitalWrite(ledPin, LOW);
  //     }
  // Check for change in raw input
  if (reading != lastButtonState) {
    lastDebounceTime = currentTime;
  }
  // If stable for debounceDelay
  if ((currentTime - lastDebounceTime) > debounceDelay) {
    // If state has changed
    if (reading != buttonState) {
        buttonState = reading;

        // Only act on press (LOW) with cooldown
        if (buttonState == HIGH && (currentTime - lastActionTime > cooldownTime)) {
          Serial.println("Button Released!");
          //digitalWrite(ledPin, HIGH);
          lastActionTime = currentTime;
          if (mode == 0) {
            mode = lastmode;
            Serial.println("Back from the dead");
          }
          else{
            lastmode = mode;
            mode = 0;
            Serial.println("Dead");
          }
        }

        if (buttonState == HIGH) {
          Serial.println("Button Pressed!");
          //digitalWrite(ledPin, LOW);
        }
      }
    }
    lastButtonState = reading;
  
}
