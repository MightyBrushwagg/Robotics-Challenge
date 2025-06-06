#include <Servo.h>
#include <Motoron.h>

MotoronI2C mc1(0x10), mc2(0x11);

// Servo steering
#define SERVO_PIN   27
#define FRONT       90
#define SHARP_RIGHT (FRONT + 65)
#define SHARP_LEFT  (FRONT - 65)

// Speed params
#define BASE_SPEED  40
#define MAX_SPEED   60
#define MIN_SPEED   20

// initialize sensor pins
const int numIRSensors = 3;
const int IRPins[numIRSensors] = {A7, A6, A5};
unsigned int IRValues[numIRSensors];
float voltageIR[numIRSensors];
float distanceIR[numIRSensors];

Servo head;

void setup() {
  Serial.begin(9600);
  analogReadResolution(12); 
  head.attach(SERVO_PIN);
  head.write(FRONT); 
  InitializeMotorShields();
}

void loop() {
  wall_following();
}

void InitializeMotorShields() {
  Wire1.begin();
  mc1.setBus(&Wire1);
  mc2.setBus(&Wire1);

  mc1.reinitialize(); mc1.disableCrc(); mc1.clearResetFlag(); mc1.disableCommandTimeout();
  for (uint8_t ch = 1; ch <= 2; ch++) {
    mc1.setMaxAcceleration(ch, 100);
    mc1.setMaxDeceleration(ch, 100);
  }

  mc2.reinitialize(); mc2.disableCrc(); mc2.clearResetFlag(); mc2.disableCommandTimeout();
  for (uint8_t ch = 1; ch <= 2; ch++) {
    mc2.setMaxAcceleration(ch, 100);
    mc2.setMaxDeceleration(ch, 100);
  }
}


void IR_Sensor_Reading() {
  for (int i = 0; i < numIRSensors; i++) {
    IRValues[i] = analogRead(IRPins[i]);
    voltageIR[i] = IRValues[i] * (5.0 / 4095.0);
    distanceIR[i] = (-13.0 * voltageIR[i]) + 23.2;
    if (distanceIR[i] < 2) distanceIR[i] = 2;
    if (distanceIR[i] > 15) distanceIR[i] = 15;
  }
}

// wall following
void wall_following() {
  IR_Sensor_Reading();

  float frontDistance = distanceIR[0];  // A7
  float rightDistance = distanceIR[1];  // A6

  const float wallDistance = 10.0;      // ideal left/right wall dist
  const float frontThreshold = 6.0;     // front pnbstacle threshold
  const float Kp_wall = 5.0;            

  int angle = FRONT;
  int speed = BASE_SPEED;

  if (frontDistance < frontThreshold) {
    // Obstacle ahead, sharp left
    angle = SHARP_LEFT;
    speed = MIN_SPEED;
  } else {
    float error = rightDistance - wallDistance;
    int correction = Kp_wall * error;
    angle = constrain(FRONT - correction, SHARP_LEFT, SHARP_RIGHT);
    speed = constrain(BASE_SPEED - abs(correction), MIN_SPEED, MAX_SPEED);
  }

  head.write(angle);
  go_Advance_2_wheel(speed, speed);


  Serial.print("Front: "); Serial.print(frontDistance, 2);
  Serial.print("  Right: "); Serial.print(rightDistance, 2);
  Serial.print("  Angle: "); Serial.print(angle);
  Serial.print("  Speed: "); Serial.println(speed);
}

void go_Advance_2_wheel(int leftSpeed, int rightSpeed) {
  int16_t mL = constrain(leftSpeed, MIN_SPEED, MAX_SPEED) * 4;
  int16_t mR = constrain(rightSpeed, MIN_SPEED, MAX_SPEED) * 4;
  mc1.setSpeed(2, mL); mc1.setSpeed(1, mR);
  mc2.setSpeed(2, mL); mc2.setSpeed(1, mR);
}

void stop_Stop() {
  mc1.setSpeed(1, 0); mc1.setSpeed(2, 0);
  mc2.setSpeed(1, 0); mc2.setSpeed(2, 0);
}
