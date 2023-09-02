#include <Servo.h>

Servo talonSRX;

int power = 25; //In percent from 0 to 100
double encoderAngle = 0.0;
double targetEncoderAngle = 0.0;

int talonPWMPin = 9; //Pin the yellow can bus wire is connected to
int encoderPWMPin = 0; //Pin the absolute encoder is pluged into

//PID
double pastError = 0.0;
double pastInt = 0.0;
double Kp = 1.0;
double Ki = 0.0;
double Kd = 0.0;
int iterationTime = 50;

void setup() {
  talonSRX.attach(talonPWMPin);  // attaches the server object for the talon to the talonPWMPin pin
  Serial.begin(9600);
}

void loop() {
  setMotorPower(pidPowerToAngle(getAngle(), targetEncoderAngle));
  delay(iterationTime);
}

int pidPowerToAngle(double angle, double targetAngle){
  double error = targetAngle-angle;
  double i = pastInt + error * iterationTime;
  double d = (error-pastError)/iterationTime;
  double out = Kp*error+Ki*i+Kd*d;
  pastError = error;
  pastInt = i;
  int ret = constrain(int(out),-80,80);
  Serial.print(ret);
  Serial.print(" - ");
  Serial.print(error);
  Serial.print(" - ");
  Serial.println(angle);
  return ret;
}

double getAngle(){
  //Using Encoder magic to get the absolute angle of the steering column
  // return map(analogRead(A0), 0, 1023, -180, 180); //temp test code
  return encoderAngle;
}

void setMotorPower(int motorPower){
  int rawPower = map(motorPower, -100, 100, 0, 1023); //Converts percent out put to raw output for the motor
  talonSRX.write(rawPower); //writes the power to the talon srx
}