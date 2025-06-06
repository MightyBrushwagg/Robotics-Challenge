//0527 testing code
#include <Servo.h>
#include <Motoron.h>

// Motoron shields on I2C addresses 0x10 (front) and 0x11 (back)
MotoronI2C mc1(0x10), mc2(0x11);

// important control
int mode = 0; // 0 = dead, 1 = line following, 2 = wall following, 
int lastmode = 0;

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
#define MAX_SPEED   60
#define MIN_SPEED   20

// PID gains
//const float Kp = 0, Ki = 0, Kd = 0; 
const float Kp = 30, Ki = 0, Kd = 1; // 30,0,0 best
float integral = 0, previousErr = 0;
const int weight[numReflectanceSensors] = {-6,-5,-4,-3,-2,-1,0,1,2,3,4,5,6};

const int numIRSensors = 3;
const int IRPins[numIRSensors] = {A7, A6, A5};
unsigned int IRValues[numIRSensors];
float voltageIR[numIRSensors];
float distanceIR[numIRSensors];

Servo head;

// Crossroads
const int numCrossroads = 2;
const int crossroadDirections[numCrossroads] = {60,60};
int currentCrossroad = 0;
int crossroading = 0;
unsigned int startedTurn;
const int crossroadThreshold = 1000000;

//wall following
const float idealWallDistance = 10.0;     // ideal distance from wall 
const float margin = 1.5;                 // error margin

// losing line
int lastSeenSide = -1;

void setup() {
  InitializeMotorShield();
  head.attach(SERVO_PIN);
  head.write(FRONT);
  Serial.begin(9600);
  analogReadResolution(12);
}

void loop() {
  // do button and wifi check. will add code over weekend
  //button_check();
  //wifi_check();

  // 0 = dead, 1 = line following, 2 = wall following, 
  if (mode == 1) {
    if(crossroading == 0){
      auto_tracking();
      check_crossroad();
    } else if (crossroading == 1) {
    // take crossroad
      take_crossroad();
      head.write(140);
      Reflectance_Sensor_Reading();
      if (micros() - startedTurn >= crossroadThreshold) {
        // crossroading = 0;
        currentCrossroad++;
        for (int i = 0; i < numReflectanceSensors; i++) {
      Serial.print(sensorValuesBinary[i] ? '1' : '0');
    }
        Serial.println("\tTaken the crossroad");
      }
      // stop_Stop();
      go_Advance_2_wheel(BASE_SPEED+10, BASE_SPEED-10);

    }
  } else if (mode == 2) {
    wall_following();
  } else {
    stop_Stop();
  }
  
}

void take_crossroad() {
  if (crossroading == 1) {
    //do nothing as should not be triggered
  } else {
    head.write(crossroadDirections[currentCrossroad]);
    // switch (crossroadDirections[currentCrossroad]) {
      
    //   case 0:
    //     // turn right
    //     head.write(SHARP_RIGHT);
    //     go_Advance_2_wheel(BASE_SPEED+10, BASE_SPEED-10);
    //     Serial.println("right");
    //     break;

    //   case 1:
    //     // turn left
    //     head.write(SHARP_LEFT);
    //     go_Advance_2_wheel(BASE_SPEED-10, BASE_SPEED+10);
    //     Serial.println("left");
    //     break;

    //   default:
    //     go_Advance_2_wheel(BASE_SPEED, BASE_SPEED);
        
    //     break;

    // }
  }
}

void check_crossroad() {
  Reflectance_Sensor_Reading();
  crossroading = 0;

  int zeroBefore = 0;
  int zeroThenOne = 0;

  for (int i=0; i<(numReflectanceSensors-1); i++){
    if (sensorValuesBinary[i] == 0 && zeroThenOne == 0){
      zeroBefore = 1;
    } else if (sensorValuesBinary[i] == 1 && zeroBefore == 1) {
      zeroThenOne = 1;
    } else if (sensorValuesBinary[i] == 0 && zeroThenOne == 1) {
      crossroading = 1;
      startedTurn = micros();
    }
  }
}

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

  // Debug: print raw values and binary state
  for (int i = 0; i < numReflectanceSensors; i++) {
    // Serial.print(sensorValuesFull[i]); Serial.print('\t');
  }
  // Serial.print("| ");
  for (int i = 0; i < numReflectanceSensors; i++) {
    Serial.print(sensorValuesBinary[i] ? '1' : '0');
  }
  Serial.println();
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
float pid(float error) {
  unsigned long now = micros();
  float dt = lastMicros ? (now - lastMicros) / 1e6 : 0.01;  
  lastMicros = now;

  integral    += error * dt;
  float derivative = (error - previousErr) / dt;
  previousErr = error;
  return Kp * error + Ki * integral + Kd * derivative;
}

// Line-following with front-wheel PID steering
void auto_tracking() {
  Reflectance_Sensor_Reading();
  
  int sumW = 0, sumH = 0;
  for (int i = 0; i < numReflectanceSensors; i++) {
    if (sensorValuesBinary[i]) { sumW += weight[i]; sumH++; } 
  }
  float error = (sumH>0) ? float(sumW)/sumH : 0;

  //************************DBUG*************************
  // Serial.print("Err: "); Serial.print(error, 2);
  float output = pid(error);  
  // Serial.print("  Out: "); Serial.print(output, 2);
  int angle = constrain(FRONT - int(output), SHARP_LEFT, SHARP_RIGHT);
  // Serial.print("  Angle: "); Serial.println(angle);

  // Steering output: map PID output to servo angle
  //int angle = constrain(FRONT - int(output), SHARP_LEFT, SHARP_RIGHT);
  
  // Serial.println(angle);
  //delay(150);
  

  //Compute weighted centroid error using raw values
  
  // Speed output: reduce speed if steering correction is large
  int speedBase = constrain(BASE_SPEED - abs(int(output)) * 3, MIN_SPEED, MAX_SPEED);
  int leftSpd   = speedBase - int(output * 1.5);
  int rightSpd  = speedBase + int(output * 1.5);
  // Serial.println(leftSpd);
  // Serial.println(rightSpd);

  

  // Special cases: lost line or finish line
  if (sumH == numReflectanceSensors) {
    // Line lost: center wheels and proceed slowly

    if (lastSeenSide == -1) {
      head.write(FRONT);
      int speed = MIN_SPEED;
      go_Advance_2_wheel(MIN_SPEED, MIN_SPEED);
    } else if (lastSeenSide == 1) {
      head.write(110);
      int speed = BASE_SPEED;
      go_Advance_2_wheel(BASE_SPEED-10, BASE_SPEED+10);
    } else if (lastSeenSide == 2) {
      head.write(70);
      int speed = BASE_SPEED;
      go_Advance_2_wheel(BASE_SPEED+10, BASE_SPEED-10);
    }

    head.write(FRONT);
    int speed = MIN_SPEED;
    go_Advance_2_wheel(MIN_SPEED, MIN_SPEED);
  }
  else if (sumH == 0) {
    // All sensors on line: assume finish line, stop
    stop_Stop();
    head.write(FRONT);
    return;
  }
  else {
    if (sumH == numReflectanceSensors-2 || sumH == numReflectanceSensors-1) {
      if (sensorValuesBinary[0] == 0 || sensorValuesBinary[1] == 0) {
        lastSeenSide = 1;
      } else if (sensorValuesBinary[numReflectanceSensors-1] == 0 || sensorValuesBinary[numReflectanceSensors-2] == 0) {
        lastSeenSide = 2;
      } else {
        lastSeenSide = -1;
      }
    } else {
      lastSeenSide = -1;
    }
    go_Advance_2_wheel(leftSpd, rightSpd);
    head.write(angle);
  }
}

void wall_following() {
  IR_Sensor_Reading();

  float leftDistance = distanceIR[0];   // A2 = Left
  float rightDistance = distanceIR[1];  // A3 = Right

  // left wall check
  if ((leftDistance >= (idealWallDistance - margin) && leftDistance <= (idealWallDistance + margin)) || (rightDistance >= (idealWallDistance - margin) && rightDistance <= (idealWallDistance + margin))) {
    Serial.println("Left: OK (within ideal distance)-  drive forward");
    head.write(FRONT);
    go_Advance_2_wheel(BASE_SPEED, BASE_SPEED);
  } else if ((leftDistance > (idealWallDistance - margin)) || (rightDistance < (idealWallDistance - margin))) {
    

    Serial.println("Left: TOO FAR - steer left");
    head.write(SHARP_LEFT);
    go_Advance_2_wheel(BASE_SPEED-10, BASE_SPEED+10);
  } else {
    Serial.println("Left: TOO CLOSE - steer right");
    head.write(SHARP_RIGHT);
    go_Advance_2_wheel(BASE_SPEED+10, BASE_SPEED-10);
  }

  // right wall check
  // if (rightDistance >= (idealWallDistance - margin) && rightDistance <= (idealWallDistance + margin)) {
  //   Serial.println("Right: OK (within ideal distance)");
  //   head.write(FRONT);
  //   go_Advance_2_wheel(BASE_SPEED, BASE_SPEED);
  // } else if (rightDistance < (idealWallDistance - margin)) {
  //   Serial.println("Right: TOO CLOSE - steer left");
  //   head.write(SHARP_LEFT);
  //   go_Advance_2_wheel(BASE_SPEED-10, BASE_SPEED+10);
  // } else {
  //   Serial.println("Right: TOO FAR - steer right");
  //   head.write(SHARP_RIGHT);
  //   go_Advance_2_wheel(BASE_SPEED+10, BASE_SPEED-10);
  // }

  Serial.print("Left: "); Serial.print(leftDistance, 2);
  Serial.print(" cm\tRight: "); Serial.print(rightDistance, 2);
  Serial.println(" cm\n");
}


































