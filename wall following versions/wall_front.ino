const int IR_FRONT = A4;
unsigned int IRValue;
float voltageIR;
float distanceIR;

const float frontThreshold = 7.0;  // cm, threshold to turn right

// Motor control pins (fill in with your actual motor pins)
const int LEFT_PWM  = 5;
const int LEFT_DIR  = 4;
const int RIGHT_PWM = 6;
const int RIGHT_DIR = 7;

void setup() {
  Serial.begin(9600);

  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
}

void loop() {
  readFrontIR();

  Serial.print("Front Distance: ");
  Serial.print(distanceIR, 1);
  Serial.println(" cm");

  if (distanceIR <= frontThreshold) {
    Serial.println("Wall ahead — turning RIGHT");
    turnRight();
    delay(800);   // adjust for a 90° turn
    stopMotors();
  } else {
    driveForward();
  }

  delay(50); // short delay for sensor stability
}

void readFrontIR() {
  IRValue = analogRead(IR_FRONT);
  voltageIR = IRValue * (5.0 / 4095.0);  // for 12-bit ADC (like Nicla Vision)
  distanceIR = (-13.0 * voltageIR) + 23.2;

  if (distanceIR < 2.0)  distanceIR = 2.0;
  if (distanceIR > 15.0) distanceIR = 15.0;
}

void driveForward() {
  analogWrite(LEFT_PWM, 60);
  digitalWrite(LEFT_DIR, HIGH);
  analogWrite(RIGHT_PWM, 60);
  digitalWrite(RIGHT_DIR, HIGH);
}

void stopMotors() {
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
}

void turnRight() {
  // Left motor forward, right motor backward
  analogWrite(LEFT_PWM, 60);
  digitalWrite(LEFT_DIR, HIGH);
  analogWrite(RIGHT_PWM, 60);
  digitalWrite(RIGHT_DIR, LOW);
}
