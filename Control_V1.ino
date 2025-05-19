#include <PWMServo.h>

// Definition
#define IN1        7 //********CHANGE*************
#define IN2        8  //********CHANGE*************
#define ENA        5 

#define SERVO_PIN  9    //********CHANGE*************

// Servo center position and steering limits
#define FRONT        90
#define SHARP_RIGHT  (FRONT + 33) //********CHANGE*************
#define SHARP_LEFT   (FRONT - 40) //********CHANGE*************

// Line-following sensor pins
#define LFSensor_0   A0
#define LFSensor_1   A1
#define LFSensor_2   A2
#define LFSensor_3   A3
#define LFSensor_4   A4
#define LFSensor_5   A5
#define LFSensor_6   A6
#define LFSensor_7   A7
#define LFSensor_8   A8

// Speed parameters
#define BASE_SPEED   150 //********CHANGE*************
#define MAX_SPEED    200 //********CHANGE*************
#define MIN_SPEED    100 //********CHANGE*************

// PID tuning parameters
const float Kp = 20.0;  //********CHANGE*************
const float Ki = 0.5;  //********CHANGE*************
const float Kd = 8.0;   //********CHANGE*************

// PID state variables
float integral    = 0;
float previousErr = 0;

// Buffer to store sensor readings
bool sensor[9];

// Servo object for steering
PWMServo head;

void setup() {
  // Initialize motor driver pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  stop_Stop();

  // Attach and center the steering servo
  head.attach(SERVO_PIN);
  head.write(FRONT);

  // Begin serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  auto_tracking();  // Main line-following routine
}

// Drive the motor forward at the specified speed
void go_Advance(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);
}

// Stop the motor
void stop_Stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}

// Line-following with front-wheel PID steering
void auto_tracking() {
  // Read sensors: 1 = line detected, 0 = no line
  sensor[0] = !digitalRead(LFSensor_0);
  sensor[1] = !digitalRead(LFSensor_1);
  sensor[2] = !digitalRead(LFSensor_2);
  sensor[3] = !digitalRead(LFSensor_3);
  sensor[4] = !digitalRead(LFSensor_4);
  sensor[5] = !digitalRead(LFSensor_5);
  sensor[6] = !digitalRead(LFSensor_6);
  sensor[7] = !digitalRead(LFSensor_7);
  sensor[8] = !digitalRead(LFSensor_8);

  // Debug: print binary sensor state
  int raw = sensor[0]*256 + sensor[1]*128 + sensor[2]*64 + sensor[3]*32 + sensor[4]*16 + sensor[5]*8 + sensor[6]*4 + sensor[7]*2 + sensor[8];
  String s = String(raw, BIN);
  while (s.length() < 9) s = "0" + s;
  Serial.print("SENSOR="); Serial.println(s);

  // Calculate error: weighted average of sensor positions
  const int weight[9] = {-4, -3, -2, -1, 0, 1, 2, 3, 4};
  int sumWeight = 0, sumHits = 0;
  for (int i = 0; i < 9; i++) {
    if (sensor[i]) {
      sumWeight += weight[i];
      sumHits++;
    }
  }
  // If no sensor detects line or all sensors detect line, handle later
  float position = (sumHits > 0) ? (float)sumWeight / sumHits : 0;
  float error = position;  // Desired position is 0 (center)

  // PID computation
  integral    += error;
  float derivative = error - previousErr;
  float output     = Kp * error + Ki * integral + Kd * derivative;
  previousErr = error;

  // Steering output: map PID output to servo angle
  int angle = FRONT + (int)output;
  angle = constrain(angle, SHARP_LEFT, SHARP_RIGHT);
  head.write(angle);

  // Speed output: reduce speed if steering correction is large
  int speed = BASE_SPEED - abs((int)output) * 3;
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);

  // Special cases: lost line or finish line
  if (sumHits == 0) {
    // Line lost: center wheels and proceed slowly
    head.write(FRONT);
    speed = MIN_SPEED;
    go_Advance(speed);
  }
  else if (sumHits == 9) {
    // All sensors on line: assume finish line, stop
    stop_Stop();
    head.write(FRONT);
    return;
  }
  else {
    go_Advance(speed);
  }
}

