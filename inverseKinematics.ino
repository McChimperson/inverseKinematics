#include "Arduino.h"
#include <Servo.h>
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;
Servo servo7;
Servo servo8;




float distance(float x1, float y1, float x2, float y2) {
  float result = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
  return result;
}


float angle(float x1, float y1, float x2, float y2) {
  float result = atan2(y2 - y1, x2 - x1);
  return result;
}

float* circleFind(float x1, float y1, float r1, float x2, float y2, float r2, float sign) {

  float D = distance(x1, y1, x2, y2);
  float xr = (pow(r1, 2) - pow(r2, 2) + pow(D, 2)) / (2 * D);
  float yr = sign * sqrt(pow(r1, 2) - pow(xr, 2));
  float thetaR = atan2(yr, xr);

  float thetaD = angle(x1, y1, x2, y2);
  float thetaT = thetaD - thetaR;
  float x3 = r1 * cos(thetaT) + x1;
  float y3 = r1 * sin(thetaT) + y1;
  
  static float xy[2] = {x3,y3};
  return xy;
}

float* IKSERVOS (float x3 = 0.5, float y3 = -5)
{
  x3 += .5; //Makes it act like x=0 at crotch center

  float x1 = 0.0;         //back hip perm location
  float x2 = 1.0;         //front hip perm location
  //float x3 = 0.5;    //assigned foot location
  float x4;                 //back knee solved
  float x5;                 //front knee solved
  float x6 = 0.0;         //back servo perm location
  float x7 = 1.0;         //front servo perm location
  float x8;                 //rear springtop solved
  float x9;                 //front springtop solved

  float y1 = 0.0;         //back hip perm location
  float y2 = 0.0;         //front hip perm location
  //float y3 = -4;   //assigned foot location
  float y4;                 //back knee solved
  float y5;                 //front knee solved
  float y6 = 1.25;      //back servo perm location
  float y7 = 1.25;      //front servo perm location
  float y8;                 //rear springtop solved
  float y9;                 //front springtop solved

  float r1 = 3.5; //back thigh length
  float r2 = 3.5; //front thigh length
  //float r3 = 0; //nothing to assign here
  float r4 = 4.0; //backcalf length
  float r5 = 4.0; //frontcalf length
  float r6 = 1.0; //backcrank length
  float r7 = 1.0; //frontcrank length
  float r8 = 4.5; //frontspring length  (kinda fudged)
  float r9 = 4.5; //backspring length  (kinda fudged)
  float sign; //determines is top or bottom solution is chosed for circle intersection



  float* xy4;
  float* xy5;
  float* xy8;
  float* xy9;
  
  xy4 = circleFind(x1, y1, r1, x3, y3, r4, 1);
  x4 = xy4[0]; //
  y4 = xy4[1];

  xy5 = circleFind(x2, y2, r2, x3, y3, r5, -1);
  x5 = xy5[0];
  y5 = xy5[1];
  
  xy8 = circleFind(x6, y6, r6, x4, y4, r8, 1);
  x8 = xy8[0];
  y8 = xy8[1];

  xy9 = circleFind(x7, y7, r7, x5, y5, r9, -1);
  x9 = xy9[0];
  y9 = xy9[1];

  float thetas1 = angle(x6, y6, x8, y8);
  float thetas2 = angle(x7, y7, x9, y9);

  float backservoangle = 180 * thetas1 / PI - 45;
  float frontservoangle = 180 * thetas2 / PI + 45;

  if (backservoangle < 0)
  {
    backservoangle += 360;
  }
  if (frontservoangle < 0)
  {
    frontservoangle += 360;
  }

  static float servoAngles[2] = {frontservoangle,backservoangle};
  return servoAngles;
}



void setup() {
  servo1.attach(30);  // attaches the servo on pin 30 to the servo object
  servo2.attach(31);  // attaches the servo on pin 31 to the servo object
  servo3.attach(32);  // attaches the servo on pin 32 to the servo object
  servo4.attach(33);  // attaches the servo on pin 33 to the servo object
  servo5.attach(34);  // attaches the servo on pin 34 to the servo object
  servo6.attach(35);  // attaches the servo on pin 35 to the servo object
  servo7.attach(36);  // attaches the servo on pin 36 to the servo object
  servo8.attach(37);  // attaches the servo on pin 37 to the servo object

}

void loop() {

  float tstep=0.002;
  float xamp=2.3; 
  float yoffset=4.5; //coordinate system, y below hip is negative. yoffset is SUBTRACTED from the y coordinate, so it should be a positive number
  float ylift=4; //how much the foot is lifted off the ground. ylift is ADDED to the y coordinate during the liftoff phase
  float* servoAngles;
 
 //t=0
   for (float t = 0; t < 1; t += tstep)
  {
        //front left
    servoAngles = IKSERVOS(xamp*(-1.0+2.0*t),- yoffset +ylift*t-ylift*t*t);
    servo1.write(servoAngles[0]);
    servo2.write(servoAngles[1]);
        //front right
    servoAngles = IKSERVOS(xamp*(-1.0+2.0*t),-yoffset);
    servo3.write(servoAngles[0]);
    servo4.write(servoAngles[1]);
        //back right
    servoAngles = IKSERVOS(xamp*(-(-1.0+2.0*t)),- yoffset +ylift*t-ylift*t*t);
    servo5.write(servoAngles[0]);
    servo6.write(servoAngles[1]);

        //back left
    servoAngles = IKSERVOS(xamp*(1.0-2.0*t),- yoffset);
    servo7.write(servoAngles[0]);
    servo8.write(servoAngles[1]);

  }
  
  //t=1
    for (float t = 0; t < 1; t += tstep)
  {
    servoAngles = IKSERVOS(xamp*(1.0-2.0*t),- yoffset);
    servo1.write(servoAngles[0]);
    servo2.write(servoAngles[1]);
    
    servoAngles = IKSERVOS(xamp*(-(-1.0+2.0*t)),- yoffset +ylift*t-ylift*t*t);
    servo3.write(servoAngles[0]);
    servo4.write(servoAngles[1]);
    
    servoAngles = IKSERVOS(xamp*(-1.0+2.0*t),- yoffset);
    servo5.write(servoAngles[0]);
    servo6.write(servoAngles[1]);
    
    servoAngles = IKSERVOS(xamp*(-1.0+2.0*t),- yoffset +ylift*t-ylift*t*t);
    servo7.write(servoAngles[0]);
    servo8.write(servoAngles[1]);

  }
  //t=2
 
 /*
 
 
 // Creep Gait
 
  for (float t = 0; t < 1; t += tstep)
  {
        //front left
    servo1.write(IKBACKSERVO(xamp*(-1.0+2.0*t),- yoffset +ylift*t-ylift*t*t));
    servo2.write(IKFRONTSERVO(xamp*(-1.0+2.0*t),- yoffset +ylift*t-ylift*t*t));
        //front right
    servo3.write(IKFRONTSERVO(xamp*(-1.0/3.0+2.0/3.0*t),-yoffset));
    servo4.write(IKBACKSERVO(xamp*(-1.0/3.0+2.0/3.0*t),- yoffset));
        //back right
    servo5.write(IKFRONTSERVO(xamp*(1.0/3.0+2.0/3.0*t),- yoffset));
    servo6.write(IKBACKSERVO(xamp*(1.0/3.0+2.0/3.0*t),- yoffset));
        //back left
    servo7.write(IKBACKSERVO(xamp*(1.0-2.0/3.0*t),- yoffset));
    servo8.write(IKFRONTSERVO(xamp*(1.0-2.0/3.0*t),- yoffset));

  }
  
  //t=1
    for (float t = 0; t < 1; t += tstep)
  {
    
    //front left
    servo1.write(IKBACKSERVO(xamp*(1.0-(2.0/3.0)*t),- yoffset));
    servo2.write(IKFRONTSERVO(xamp*(1.0-(2.0/3.0)*t),- yoffset));
    
    //front right
    servo3.write(IKFRONTSERVO(xamp*(1.0/3.0+2.0/3.0*t),-yoffset));
    servo4.write(IKBACKSERVO(xamp*(1.0/3.0+2.0/3.0*t),- yoffset));
    
    //back right
    servo5.write(IKFRONTSERVO(xamp*(-(-1.0+2.0*t)),- yoffset +ylift*t-ylift*t*t));
    servo6.write(IKBACKSERVO(xamp*(-(-1.0+2.0*t)),- yoffset +ylift*t-ylift*t*t));
    
    //back left
    servo7.write(IKBACKSERVO(xamp*((1.0/3.0)-(2.0/3.0)*t),- yoffset));
    servo8.write(IKFRONTSERVO(xamp*((1.0/3.0)-(2.0/3.0)*t),- yoffset));

  }
  //t=2
    for (float t = 0; t < 1; t += tstep)
  {
    
    servo1.write(IKBACKSERVO(xamp*(1.0/3.0-2.0/3.0*t),- yoffset));
    servo2.write(IKFRONTSERVO(xamp*(1.0/3.0-2.0/3.0*t),- yoffset));
    
    servo3.write(IKFRONTSERVO(xamp*(-(-1.0+2.0*t)),- yoffset +ylift*t-ylift*t*t));
    servo4.write(IKBACKSERVO(xamp*(-(-1.0+2.0*t)),- yoffset +ylift*t-ylift*t*t));
    
    servo5.write(IKFRONTSERVO(xamp*(-1.0+(2.0/3.0)*t),- yoffset));
    servo6.write(IKBACKSERVO(xamp*(-1.0+(2.0/3.0)*t),- yoffset));
    
    servo7.write(IKBACKSERVO(xamp*(-1.0/3.0-2.0/3.0*t),- yoffset));
    servo8.write(IKFRONTSERVO(xamp*(-1.0/3.0-2.0/3.0*t),- yoffset));

  }
  //t=3
    for (float t = 0; t < 1; t += tstep)
  {
    
    servo1.write(IKBACKSERVO(xamp*(-1.0/3.0-2.0/3.0*t),- yoffset));
    servo2.write(IKFRONTSERVO(xamp*(-1.0/3.0-2.0/3.0*t),- yoffset));
    
    servo3.write(IKFRONTSERVO(xamp*(-1.0+2.0/3.0*t),-yoffset));
    servo4.write(IKBACKSERVO(xamp*(-1.0+2.0/3.0*t),-yoffset));
    
    servo5.write(IKFRONTSERVO(xamp*(-1.0/3.0+2.0/3.0*t) , - yoffset));
    servo6.write(IKBACKSERVO(xamp*(-1.0/3.0+2.0/3.0*t) , - yoffset));
    
    servo7.write(IKBACKSERVO(xamp*(-1.0+2.0*t),- yoffset +ylift*t-ylift*t*t));
    servo8.write(IKFRONTSERVO(xamp*(-1.0+2.0*t),- yoffset +ylift*t-ylift*t*t));

  }
  //t=4
*/
//servo calibration  
/*
    servo1.write(90+45);
    servo2.write(90-45);

    servo3.write(90-45);
    servo4.write(90+45);

    servo5.write(90-45);
    servo6.write(90+45);
    
    servo7.write(90+45);
    servo8.write(90-45);
*/
}

