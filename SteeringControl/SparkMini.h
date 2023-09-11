#include <Servo.h>

class Spark{
  Servo spark;
  double speed = 0.0;

  Spark(int port){
    spark.attach(port);
  }

  void update(){
    spark.write(speed);
  }

  void set(double _speed){
    speed = _speed;
  }
}