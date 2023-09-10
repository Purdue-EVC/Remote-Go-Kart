#include <Servo.h>

Servo talonSRX;

// int power = ; //In percent from 0 to 100

int talonPWMPin = 9;  //Pin the yellow can bus wire is connected to

void setup() {
  talonSRX.attach(talonPWMPin);  // attaches the server object for the talon to the talonPWMPin pin
  Serial.begin(9600);
}

void loop() {
  int rawPower = map(0, -100, 100, 0, 1023); //Converts percent out put to raw output for the motor
  talonSRX.write(rawPower);  //writes the power to the talon srx
  Serial.println(GetPWM(3));
  // GetPWM(3);
}


int GetPWM(int pin)
{
  unsigned long highTime = pulseIn(pin, HIGH, 50000UL);  // 50 millisecond timeout
  return map(highTime, 1000, 2000, 0, 1023);  // highTime as percentage of total cycle time
}