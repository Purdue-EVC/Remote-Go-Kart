#include <Servo.h>

Servo talonSRX;

int power = 25; //In percent from 0 to 100

int talonPWMPin = 9; //Pin the yellow can bus wire is connected to

void setup() {
  talonSRX.attach(talonPWMPin);  // attaches the server object for the talon to the talonPWMPin pin
}

void loop() {
  int rawPower = map(power, 0, 1023, 0, 100); //Converts percent out put to raw output for the motor
  talonSRX.write(rawPower); //writes the power to the talon srx
}