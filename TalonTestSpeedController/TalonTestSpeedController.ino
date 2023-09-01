#include <Servo.h>

Servo talonSRX;

int power = 25; //In percent from 0 to 100

int talonPWMPin = 9;

void setup() {
  myservo.attach(talonPWMPin);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  int rawPower = map(power, 0, 1023, 0, 100);
  myservo.write(rawPower);
}