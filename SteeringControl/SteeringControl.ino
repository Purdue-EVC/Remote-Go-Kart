#include <Servo.h>
#include <Filter.h>

Servo talonSRX;

int power = 25; //In percent from 0 to 100
double encoderAngle = 0.0;
double targetEncoderAngle = 0.0;

int talonPWMPin = 9; //Pin the yellow can bus wire is connected to
int encoderPWMPin = 3; //Pin the absolute encoder is pluged into

/*
The implementation I made myself is still here; Delete if the library works - Siddarth
*/
long lastPWM = 0;
long currentPWM = 0;
long filteredPWM = 0;

float weight = 50;

ExponentialFilter<float> FilteredTemperature(weight, 0);

//PID
// int iterationTime = 50;
// double pastError = 0.0;
// double pastInt = 0.0;
// double Kp = 1.0;
// double Ki = 0.1*(1/iterationTime;
// double Kd = 0.0;

void setup() {
  talonSRX.attach(talonPWMPin);  // attaches the server object for the talon to the talonPWMPin pin
  Serial.begin(9600);
}

void loop() {
  // setMotorPower(pidPowerToAngle(getAngle(), targetEncoderAngle));
  // delay(iterationTime);
  lastPWM = currentPWM;
  currentPWM = GetPWM(encoderPWMPin);
  filteredPWM = FilteredTemperature.Filter(currentPWM);

  Serial.println(filteredPWM); //Filtered PWM
}

// int pidPowerToAngle(double angle, double targetAngle){
//   double error = targetAngle-angle;
//   double i = pastInt + error * iterationTime;
//   double d = (error-pastError)/iterationTime;
//   double out = Kp*error+Ki*i+Kd*d;
//   pastError = error;
//   pastInt = i;
//   int ret = constrain(int(out),-80,80);
//   Serial.print(ret);
//   Serial.print(" - ");
//   Serial.print(error);
//   Serial.print(" - ");
//   Serial.println(angle);
//   return ret;
// }

// long getAngle(){
//   //Using Encoder magic to get the absolute angle of the steering column
//   // return map(analogRead(A0), 0, 1023, -180, 180); //temp test code
//   return GetPWM(encoderPWMPin);
// }

// void setMotorPower(int motorPower){
//   int rawPower = map(motorPower, -100, 100, 0, 1023); //Converts percent out put to raw output for the motor
//   talonSRX.write(rawPower); //writes the power to the talon srx
// }

long GetPWM(int pin)
{
  unsigned long highTime = pulseIn(pin, HIGH, 50000UL);  // 50 millisecond timeout
  unsigned long lowTime = pulseIn(pin, LOW, 50000UL);  // 50 millisecond timeout

  // pulseIn() returns zero on timeout
  // if (highTime == 0 || lowTime == 0)
  //   return (long)digitalRead(pin) ? 100.0 : 0.0;  // HIGH == 100%,  LOW = 0%

  return (100*highTime) / (highTime + lowTime));  // highTime as percentage of total cycle time
}

long GetFilteredPWM(long currentPWM, long lastPWM, float weight) {
  return weight * currentPWM + (1 - weight) * lastPWM -1;
}