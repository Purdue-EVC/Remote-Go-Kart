#include <Servo.h>

Servo motor;

double encoderAngle = 0.0;
double targetEncoderAngle = 0.0;
const int deadRangeLow = 84;   // Low filter range
const int deadRangeHigh = 94;  // High filter range

int ch1Pin = 3;  //RC Radio Pin
int encoderPWMPin = 11;
int motorPin = 8;

long prevRawSteering = 0;
long currentRawSteering = 0;
long filteredSteering = 0;
long motorPower = 90;

float weight = 0.05;
int filterAvgNum = 5;
double avg = 0;

// PID
int iterationTime = 50;
double pastError = 0.0;
double pastInt = 0.0;
double Kp = .5;
double Ki = 0.1*(1/iterationTime);
double Kd = 0.0;

void setup() {
  motor.attach(motorPin);
  // Serial.begin(9600);
}

void serialOut(int ret, double error, double angle){
  Serial.print(ret);
  Serial.print(" - ");
  Serial.print(error);
  Serial.print(" - ");
  Serial.println(angle);
}

void loop() {
  prevRawSteering = currentRawSteering;
  currentRawSteering = approxRollingAverage(prevRawSteering, getPWM(encoderPWMPin));
  motor.write(pidPowerToAngle(getAngle(), getSteeringPower(getFilteredSteering(currentRawSteering, prevRawSteering, weight))));
}

int pidPowerToAngle(double angle, double targetAngle){
  double error = targetAngle-angle;
  double i = pastInt + error * iterationTime;
  double d = (error-pastError)/iterationTime;
  double out = Kp*error+Ki*i+Kd*d;
  pastError = error;
  pastInt = i;
  int ret = constrain(int(out),-80,80);//
  ret = map(ret,-80,80, -10, 10);
  ret = map(ret,-10,10, -40, 40);
  // serialOut(ret, error, angle);
  return ret;
}

long getAngle(){
  //Using Encoder magic to get the absolute angle of the steering column
  return map(getPWM(11), 0, 1023, -180, 180);
}

long getPWM(int pin) {
  unsigned long highTime = pulseIn(pin, HIGH, 50000UL);
  unsigned long lowTime = pulseIn(pin, LOW, 50000UL);
  return highTime;
}

long getFilteredSteering(long currentRawSteering, long prevRawSteering, float weight) {
  return weight * currentRawSteering + (1 - weight) * prevRawSteering - 1;
}

long getSteeringPower(int raw) {
  long pow = constrain(map(raw, 1190, 1990, 0, 180), 0, 180); 
  if (pow < deadRangeLow) {
    return pow;
  }
  else if (pow > deadRangeHigh) {
    return pow;
  }
  else {
    return 92;
  }
}

double approxRollingAverage (double avg, double new_sample) {
    avg -= avg / filterAvgNum;
    avg += new_sample / filterAvgNum;

    return avg;
}