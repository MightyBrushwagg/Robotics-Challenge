#include <Servo.h>
#include <Motoron.h>

// Definition
MotoronI2C mc1(0x10);  // First Motoron (default address)
MotoronI2C mc2(0x11);  // Second Motoron (custom address)

#define SERVO_PIN  9    //********CHANGE*************

// Servo center position and steering limits
#define FRONT        90
#define SHARP_RIGHT  (FRONT + 33) //********CHANGE*************
#define SHARP_LEFT   (FRONT - 40) //********CHANGE*************

// Line-following sensor pins
#define LFSensor_0   31
#define LFSensor_1   30
#define LFSensor_2   33
#define LFSensor_3   32
#define LFSensor_4   35
#define LFSensor_5   34
#define LFSensor_6   37
#define LFSensor_7   36
#define LFSensor_8   39
const int numSensors = 9;
const int sensorPins[numSensors] = {31,30,33,32,35,34,37,36,39}; // change accordingly
unsigned int sensorValues[numSensors];

// Speed parameters
#define BASE_SPEED   1000 //********CHANGE*************
#define MAX_SPEED    2800 //********CHANGE*************
#define MIN_SPEED    500 //********CHANGE*************

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
Servo myservo;

void setup() {
  // Initialize motor driver pins
  Serial.begin(9600);
  Wire1.begin();

  mc1.setBus(&Wire1);
  mc2.setBus(&Wire1);

  // Initialize first shield (motors 1-3)
  mc1.reinitialize();
  mc1.disableCrc();
  mc1.clearResetFlag();
  mc1.disableCommandTimeout();

  for (uint8_t ch = 1; ch <= 2; ch++) {
    mc1.setMaxAcceleration(ch, 100);
    mc1.setMaxDeceleration(ch, 100);
    mc2.setMaxAcceleration(ch, 100);
    mc2.setMaxDeceleration(ch, 100);
  }

  // Initialize second shield (motor 4 on channel 1)
  mc2.reinitialize();
  mc2.disableCrc();
  mc2.clearResetFlag();
  mc2.disableCommandTimeout();
  stop_Stop();

  // Attach and center the steering servo
  

  myservo.attach(9);  // attaches the servo on pin 9 to the Servo object

}

void loop() {
  auto_tracking();  // Main line-following routine
}

// Drive the motor forward at the specified speed
void go_Advance(int speed) {
  mc1.setSpeed(1,speed);
  mc1.setSpeed(2,speed);
  mc2.setSpeed(1,speed);
  mc2.setSpeed(2,speed);
}

// Stop the motor
void stop_Stop() {
  mc1.setSpeed(1,0);
  mc1.setSpeed(2,0);
  mc2.setSpeed(1,0);
  mc2.setSpeed(2,0);
}

void steer_Servo(int angle) {
  for (int pos = 0; pos <= angle; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15 ms for the servo to reach the position
  }
}



// Line-following with front-wheel PID steering
void auto_tracking() {
  // Read sensors: 1 = line detected, 0 = no line

  unsigned long startTime = micros();

  for (int i = 0; i < numSensors; i++) {
    pinMode(sensorPins[i], INPUT); // Starts discharge
  }

  // Measure discharge time
  bool done[numSensors] = {false};
  unsigned long timeout = 500; // Threshold for black line
  while (micros() - startTime < timeout) {
    for (int i = 0; i < numSensors; i++) {
      if (!done[i] && digitalRead(sensorPins[i]) == LOW) {
        sensor[i] = 0;
        done[i] = true;
      } 
    }
  }

    // Assign timeout value to any still-high sensors
  for (int i = 0; i < numSensors; i++) {
    if (!done[i]) {
      sensor[i] = 1;
    }
  }

  //   // Print the results
  // for (int i = 0; i < numSensors; i++) {
  //   Serial.print('\t');
  //   Serial.print(sensor[i]);
  //   Serial.print('\t');
  // }
  // Serial.println();



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
  steer_Servo(angle);

  // Speed output: reduce speed if steering correction is large
  int speed = BASE_SPEED - abs((int)output) * 3;
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);

  // Special cases: lost line or finish line
  if (sumHits == 0) {
    // Line lost: center wheels and proceed slowly
    steer_Servo(angle);
    speed = MIN_SPEED;
    go_Advance(FRONT);
  }
  else if (sumHits == 9) {
    // All sensors on line: assume finish line, stop
    stop_Stop();
    steer_Servo(FRONT);
    return;
  }
  else {
    go_Advance(speed);
  }
}
