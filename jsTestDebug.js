var iterationTime = 50;
var pastError = 0.0;
var pastInt = 0.0;
var Kp = 1.0;
var Ki = 1.0*(1/iterationTime);
var Kd = 0.0;

function pidPowerToAngle(angle, targetAngle){
    var error = targetAngle-angle;
    var i = pastInt + error * iterationTime;
    var d = (error-pastError)/iterationTime;
    var out = Kp*error+Ki*i+Kd*d;
    pastError = error;
    pastInt = i;
    var ret = constrain(out,-80,80);
    console.log(ret+" -err: "+error+" -ang: "+angle+" -i: "+i+" -d: "+d);
    return ret;
}

function constrain(num, min, max){
  const MIN = min ?? 1;
  const MAX = max ?? 20;
  const parsed = parseInt(num)
  return Math.min(Math.max(parsed, MIN), MAX)
}