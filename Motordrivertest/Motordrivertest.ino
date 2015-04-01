#include <DualVNH5019MotorShield.h>

DualVNH5019MotorShield driver;

void setup() {
  driver.init();
}

void loop() {
  driver.setM1Speed(100);
  driver.setM2Speed(100);
}
