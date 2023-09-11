// var iterationTime = 50;
// var pastError = 0.0;
// var pastInt = 0.0;
// var Kp = 1.0;
// var Ki = 1.0*(1/iterationTime);
// var Kd = 0.0;

// function pidPowerToAngle(angle, targetAngle){
//     var error = targetAngle-angle;
//     var i = pastInt + error * iterationTime;
//     var d = (error-pastError)/iterationTime;
//     var out = Kp*error+Ki*i+Kd*d;
//     pastError = error;
//     pastInt = i;
//     var ret = constrain(out,-80,80);
//     console.log(ret+" -err: "+error+" -ang: "+angle+" -i: "+i+" -d: "+d);
//     return ret;
// }

// function constrain(num, min, max){
//   const MIN = min ?? 1;
//   const MAX = max ?? 20;
//   const parsed = parseInt(num)
//   return Math.min(Math.max(parsed, MIN), MAX)
// }

var testData = [504,
  503,
  503,
  503,
  503,
  504,
  504,
  504,
  504,
  503,
  503,
  504,
  497,
  498,
  497,
  498,
  497,
  498,
  497,
  497,
  497,
  497,
  498,
  497,
  498,
  479,
  497,
  498,
  479,
  497,
  504,
  504,
  504,
  503,
  503,
  504,
  503,
  503,
  504,
  503,
  504,
  504,
  503,
  504,
  503,
  504,
  504,
  504,
  503,
  497,
  498,
  497,
  498,
  498,
  497,
  498,
  497,
  497,
  498,
  498,
  498,
  497,
  497,
  498,
  497,
  478,
  497,
  503,
  503,
  504,
  504,
  504,
  504,
  504,
  504,
  490,
  503, 
  800,
  801,
  804,
  798,
  808,
  799,
  802,
  798,
  808,
  799,
  802,
  798,
  808,
  799,
  802,
  798,
  808,
  799,
  802,
  798,
  808,
  799,
  802,
  798,
  808,
  799,
  802,
  798,
  808,
  799,
  802,
  798,
  808,
  799,
  802,
  798,
  808,
  799,
  802]
var avg = 0;
function filterData(data){
  avg = data[0];
  data.forEach(element => {
    // console.log(element);
    // console.log(approxRollingAverage(element));
    avg = approxRollingAverage(avg, element);
    console.log(avg);
  });
}


function approxRollingAverage(avg, new_sample) {
  avg -= avg / 5;
  avg += new_sample / 5;
  return avg;
}


filterData(testData);