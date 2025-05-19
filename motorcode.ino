#include <Motoron.h>

MotoronI2C mc1(0x10);  // First Motoron (default address)
MotoronI2C mc2(0x11);  // Second Motoron (custom address)

void setup() {
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

  Serial.println("4 Motor Test Starting...");
}

void loop() {
  // Control motors on first shield
  mc1.setSpeed(1, -2000);
  mc1.setSpeed(2, -2000);
  

  // Control motor on second shield
  mc2.setSpeed(1, 2000); 
  mc2.setSpeed(2, -2000);

  delay(5000);

  mc1.setSpeed(1, 2000);
  mc1.setSpeed(2, 2000);
  mc2.setSpeed(1, -2000); 
  mc2.setSpeed(2, 2000);

  delay(5000);

  mc1.setSpeed(1, 1000);
  mc1.setSpeed(2, 2000);
  mc2.setSpeed(1, -1000); 
  mc2.setSpeed(2, 2000);

  delay(5000);

  mc1.setSpeed(1, 2000);
  mc1.setSpeed(2, 1000);
  mc2.setSpeed(1, -2000); 
  mc2.setSpeed(2, 1000);

  delay(5000);

  mc1.setSpeed(1, 0);
  mc1.setSpeed(2, 0);
  mc2.setSpeed(1, 0); 
  mc2.setSpeed(2, 0);

}




// // // 
