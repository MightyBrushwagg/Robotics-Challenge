#include <WiFi.h>
#include <WiFiUdp.h>
#include <Motoron.h>
#include <Servo.h>



///WIFI///
char ssid[] = "PhaseSpaceNetwork_2.4G";    
char pass[] = "8igMacNet"; 

WiFiUDP Udp;

unsigned int localPort = 55500;  // Port to listen and send
char packetBuffer[255];         // Buffer for incoming data
IPAddress localIP;              // This device's IP
char message[] = "Hello, self!";

///MOTORS///
MotoronI2C mc1(0x10);  // First Motoron (default address)
MotoronI2C mc2(0x11);  // Second Motoron (custom address)
int performance_timer = 50000;
int start = 0;
int current = 0;


///REFLECTANCE SENSOR///
const int numSensors = 9;
const int sensorPins[numSensors] = {31,30,33,32,35,34,37,36,39}; // change accordingly
unsigned int sensorValues[numSensors];


///IR DISTANCE///
const int sensorPin1 = A7;  // Analog pin connected to Vo
float voltage1 = 0.0;
float distance_cm1 = 0.0;

const int sensorPin2 = A6;  // Analog pin connected to Vo
float voltage2 = 0.0;
float distance_cm2 = 0.0;

const int sensorPin3 = A5;  // Analog pin connected to Vo
float voltage3 = 0.0;
float distance_cm3 = 0.0;

//button code
const int buttonPin = 27;        // Button connected to digital pin 2
//const int ledPin = 13;          // Onboard LED for feedback

int buttonState = HIGH;         // Current debounced state
int lastButtonState = HIGH;     // Previous raw reading

unsigned long lastDebounceTime = 0;  // Last time the input changed
unsigned long debounceDelay = 50;    // Debounce delay
unsigned long cooldownTime = 300;    // Optional cooldown after press
unsigned long lastActionTime = 0;    // Last time action was taken


Servo myServo;
int pulse = 1400;          // Start from low end
int direction = 1;         // 1 = increase, -1 = decrease
int step = 5;              // Speed of change
int minPulse = 1300;
int maxPulse = 1600;

///KILL SWITCH///
volatile int alive = 1;

void setup() {
  // put your setup code here, to run once:
  alive = 1;
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

  // mc2.setMaxAcceleration(1, 100);
  // mc2.setMaxDeceleration(1, 100);

  // pinMode(trigPin, OUTPUT);
  // pinMode(echoPin, INPUT);

  analogReadResolution(12);

  pinMode(buttonPin, INPUT_PULLUP);  // Internal pull-up

  // Initialize with the actual state of the pin
  buttonState = digitalRead(buttonPin);
  lastButtonState = buttonState;
  myServo.attach(52);

  // Serial.print("Connecting to wifi...");
  // WiFi.begin(ssid, pass);
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print(".");
  // }

  // localIP = WiFi.localIP();
  // Serial.println("\nConnected.");
  // Serial.print("Local IP: ");
  // Serial.println(localIP);

  // // Start UDP
  // Udp.begin(localPort);
  // Serial.print("Listening on UDP port ");
  // Serial.println(localPort);

  

  // Give the UDP socket some time
  delay(1000);
  start = micros();

}

void loop() {
  int reading = digitalRead(buttonPin);
  // Serial.print(lastButtonState);
  // Serial.print(reading);
  // Serial.println(alive);

  if (alive == 1) {
    // do stuff
    // Serial.println("I'm Alive");
    int packetSize = Udp.parsePacket();
    if (packetSize > 1) {
      Serial.print("Received packet of size ");
      Serial.println(packetSize);
      Udp.read(packetBuffer, 255);
      packetBuffer[packetSize] = '\0'; // null terminate string
      Serial.print("Contents: ");
      Serial.println(packetBuffer);
      alive = 0;
    }
    
    myServo.writeMicroseconds(pulse);
    pulse += direction * step;

  // Reverse direction at bounds
    if (pulse >= maxPulse || pulse <= minPulse) {
      direction *= -1;
    }

    delay(30);

    current = micros();

    // int mode = ((current - start) / performance_timer) % 5;

    // switch(mode){
    //   case 0: // forward
    //     mc1.setSpeed(1, 2000);
    //     mc1.setSpeed(2, -2000);
    //     mc2.setSpeed(1, 2000); 
    //     mc2.setSpeed(2, -2000);
    //     break;

    //   case 1: // backward
    //     mc1.setSpeed(1, -2000);
    //     mc1.setSpeed(2, 2000);
    //     mc2.setSpeed(1, -2000); 
    //     mc2.setSpeed(2, 2000);
    //     break;

    //   case 2: // left
    //     mc1.setSpeed(1, 1000);
    //     mc1.setSpeed(2, -2000);
    //     mc2.setSpeed(1, 1000); 
    //     mc2.setSpeed(2, -2000);
    //     break;
      
    //   case 3: // right
    //     mc1.setSpeed(1, -2000);
    //     mc1.setSpeed(2, 1000);
    //     mc2.setSpeed(1, -2000); 
    //     mc2.setSpeed(2, 1000);
    //     break;

    //   case 4: // stop
    //     mc1.setSpeed(1, 0);
    //     mc1.setSpeed(2, 0);
    //     mc2.setSpeed(1, 0); 
    //     mc2.setSpeed(2, 0);
    //     break;
    // }

    mc1.setSpeed(1, 2000);
    mc1.setSpeed(2, -2000);
    mc2.setSpeed(1, 2000); 
    mc2.setSpeed(2, -2000);

    for (int i = 0; i < numSensors; i++) {
    pinMode(sensorPins[i], OUTPUT);
    digitalWrite(sensorPins[i], HIGH);
    }

    delayMicroseconds(10); // Give time to charge

    // Start timer and switch to input to begin discharge
    unsigned long startTime = micros();

    for (int i = 0; i < numSensors; i++) {
      pinMode(sensorPins[i], INPUT); // Starts discharge
    }

    // Measure discharge time
    bool done[numSensors] = {false};
    unsigned long timeout = 3000; // Max wait time in µs
    while (micros() - startTime < timeout) {
      for (int i = 0; i < numSensors; i++) {
        if (!done[i] && digitalRead(sensorPins[i]) == LOW) {
          sensorValues[i] = micros() - startTime;
          done[i] = true;
        }
      }
    }

    // Assign timeout value to any still-high sensors
    for (int i = 0; i < numSensors; i++) {
      if (!done[i]) {
        sensorValues[i] = timeout;
      }
    }

    // Print the results
    for (int i = 0; i < numSensors; i++) {
      Serial.print('\t');
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
    Serial.println();

    int sensorValue1 = analogRead(sensorPin1);

    // Convert ADC value (0–4095) to voltage (0–5V)
    voltage1 = sensorValue1 * (5.0 / 4095.0);

    // Approximate distance from voltage based on datasheet
    // From Sharp's graph, at 0.4V = 15cm, at 1.65V = 2cm
    // Reverse linear interpolation between those two:
    distance_cm1 = (-13.0 * voltage1) + 23.2;

    // Clamp output to valid range
    if (distance_cm1 < 2) distance_cm1 = 2;
    if (distance_cm1 > 15) distance_cm1 = 15;

    // Serial.print("Voltage1: ");
    // Serial.print(voltage1, 2);
    Serial.print("\tDistance1: ");
    Serial.print(distance_cm1, 2);
    // Serial.println(" cm");

    int sensorValue2 = analogRead(sensorPin2);

    // Convert ADC value (0–4095) to voltage (0–5V)
    voltage2 = sensorValue2 * (5.0 / 4095.0);

    // Approximate distance from voltage based on datasheet
    // From Sharp's graph, at 0.4V = 15cm, at 1.65V = 2cm
    // Reverse linear interpolation between those two:
    distance_cm2 = (-13.0 * voltage2) + 23.2;

    // Clamp output to valid range
    if (distance_cm2 < 2) distance_cm2 = 2;
    if (distance_cm2 > 15) distance_cm2 = 15;

    // Serial.print("Voltage2: ");
    // Serial.print(voltage2, 2);
    Serial.print("\tDistance2: ");
    Serial.print(distance_cm2, 2);
    // Serial.println(" cm");

    int sensorValue3 = analogRead(sensorPin3);

    // Convert ADC value (0–4095) to voltage (0–5V)
    voltage3 = sensorValue3 * (5.0 / 4095.0);

    // Approximate distance from voltage based on datasheet
    // From Sharp's graph, at 0.4V = 15cm, at 1.65V = 2cm
    // Reverse linear interpolation between those two:
    distance_cm3 = (-13.0 * voltage3) + 23.2;

    // Clamp output to valid range
    if (distance_cm3 < 2) distance_cm3 = 2;
    if (distance_cm3 > 15) distance_cm3 = 15;

    // Serial.print("Voltage3: ");
    // Serial.print(voltage3, 2);
    Serial.print("\tDistance3: ");
    Serial.print(distance_cm3, 2);
    // Serial.println(" cm");

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
          if (alive == 0) {
            alive = 1;
            Serial.println("Back from the dead");
          }
          else if (alive == 1){
            alive = 0;
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

    
  } else if (alive == 0) {
    //do nothing
    mc1.setSpeed(1, 0);
    mc1.setSpeed(2, 0);
    mc2.setSpeed(1,0);
    mc2.setSpeed(2,0);

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
          if (alive == 0) {
            alive = 1;
            Serial.println("Back from the dead");
          }
          else if (alive == 1){
            alive = 0;
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
}
















