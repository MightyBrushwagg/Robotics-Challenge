//0521 testing code
#include <Servo.h>
#include <Motoron.h>

// Motoron shields on I2C addresses 0x10 and 0x11
MotoronI2C mc1(0x10), mc2(0x11);

// Reflectance sensors (9 channels)
const int numSensors = 13;
const int sensorPins[numSensors] = {29,31,30,33,32,35,34,37,36,39,41,40,43};
unsigned int sensorValues[numSensors];
bool sensor[numSensors];
const unsigned int reflectanceThreshold = 300; 

// Steering servo
#define SERVO_PIN   28
#define FRONT       90
#define SHARP_RIGHT (FRONT + 65)
#define SHARP_LEFT  (FRONT - 65)

// Speed params
#define BASE_SPEED  40
#define MAX_SPEED   60
#define MIN_SPEED   20

// PID gains
const float Kp = 19, Ki = 0, Kd = 15; 
float integral = 0, previousErr = 0;

Servo head;

// Prototypes
void InitializeMotorShield();
void Sensor_Reading();
void go_Advance(int leftSpeed, int rightSpeed);
void stop_Stop();
void auto_tracking();

void setup() {
  InitializeMotorShield();
  head.attach(SERVO_PIN);
  head.write(FRONT);
  Serial.begin(9600);
}

void loop() {
  auto_tracking();
}

void InitializeMotorShield() {
  Wire1.begin();
  mc1.setBus(&Wire1);
  mc2.setBus(&Wire1);

  // First shield (channels 1-2)
  mc1.reinitialize(); mc1.disableCrc(); mc1.clearResetFlag(); mc1.disableCommandTimeout();
  for (uint8_t ch = 1; ch <= 2; ch++) {
    mc1.setMaxAcceleration(ch,100);
    mc1.setMaxDeceleration(ch,100);
  }
  // Second shield (if used)
  mc2.reinitialize(); mc2.disableCrc(); mc2.clearResetFlag(); mc2.disableCommandTimeout();
  for (uint8_t ch = 1; ch <= 2; ch++) {
    mc2.setMaxAcceleration(ch,100);
    mc2.setMaxDeceleration(ch,100);
  }
}

void Sensor_Reading() {
  // 1) Charge cycle
  for (int i = 0; i < numSensors; i++) {
    pinMode(sensorPins[i], OUTPUT);
    digitalWrite(sensorPins[i], HIGH);
  }
  delayMicroseconds(10);

  // 2) Begin discharge
  unsigned long startTime = micros();
  for (int i = 0; i < numSensors; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // 3) Measure discharge time
  bool done[numSensors] = {false};
  unsigned long timeout = 3000;
  while (micros() - startTime < timeout) {
    for (int i = 0; i < numSensors; i++) {
      if (!done[i] && digitalRead(sensorPins[i]) == LOW) {
        sensorValues[i] = micros() - startTime;
        done[i] = true;
      }
    }
  }
  for (int i = 0; i < numSensors; i++) {
    if (!done[i]) sensorValues[i] = timeout;
  }
}

void go_Advance(int leftSpeed, int rightSpeed) {
  int16_t mL = constrain(leftSpeed, MIN_SPEED, MAX_SPEED) * 4;
  int16_t mR = constrain(rightSpeed, MIN_SPEED, MAX_SPEED) * 4;
  mc1.setSpeed(2, mL);
  mc1.setSpeed(1, mR);
  mc2.setSpeed(2, mL);
  mc2.setSpeed(1, mR);
}


void stop_Stop() {
  mc1.setSpeed(1, 0);
  mc1.setSpeed(2, 0);
  mc2.setSpeed(1, 0);
  mc2.setSpeed(2, 0);
}

// Line-following with front-wheel PID steering
void auto_tracking() {
  Sensor_Reading();

  //Convert to binary states based on threshold
  for (int i = 0; i < numSensors; i++) {
    sensor[i] = (sensorValues[i] < reflectanceThreshold);
  }

  // Debug: print raw values and binary state
  for (int i = 0; i < numSensors; i++) {
    Serial.print(sensorValues[i]); Serial.print('\t');
  }
  Serial.print("| ");
  for (int i = 0; i < numSensors; i++) {
    Serial.print(sensor[i] ? '1' : '0');
  }
  Serial.println();

  // Calculate error: weighted average of sensor positions
  const int weight[numSensors] = {-6,-5-4,-3,-2,-1,0,1,2,3,4,5,6};
  int sumW = 0, sumH = 0;
  for (int i = 0; i < numSensors; i++) {
    if (sensor[i]) { sumW += weight[i]; sumH++; }
  }
  float error = (sumH>0) ? float(sumW)/sumH : 0;


  // 5) PID
  integral    += error;
  float derivative = error - previousErr;
  float output     = Kp*error + Ki*integral + Kd*derivative;
  previousErr = error;

  // Steering output: map PID output to servo angle
  int angle = constrain(FRONT - int(output), SHARP_LEFT, SHARP_RIGHT);
  head.write(angle);
  Serial.println(angle);
  //delay(150);
  

  // Speed output: reduce speed if steering correction is large
  int speedBase = constrain(BASE_SPEED - abs(int(output)) * 3, MIN_SPEED, MAX_SPEED);
  int leftSpd   = speedBase - int(output * 1.5);
  int rightSpd  = speedBase + int(output * 1.5);
  Serial.println(leftSpd);
  Serial.println(rightSpd);

  // Special cases: lost line or finish line
  if (sumH == 0) {
    // Line lost: center wheels and proceed slowly
    head.write(FRONT);
    int speed = MIN_SPEED;
    go_Advance(MIN_SPEED, MIN_SPEED);
  }
  else if (sumH == 9) {
    // All sensors on line: assume finish line, stop
    stop_Stop();
    head.write(FRONT);
    return;
  }
  else {
    go_Advance(leftSpd, rightSpd);
  }
}
