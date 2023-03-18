#define maxSPD 9900 // maximum speed of motor in degs/sec

long vel = 0;
int dacOut = 0;
unsigned long t = 0;
unsigned long tOffset = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  tOffset = micros();
  while (t < 10*pow(10,6)) {
    vel = velocityOut(3600, 10*pow(10,6), t); // 3600 degs, 10 seconds (10*pow(10,6) microseconds), output in degs/s
    dacOut = map(vel, 0, maxSPD, 0, 255); // Convert from deg/sec to dac output value
    t = micros() - tOffset;
    Serial.print(t);
    Serial.print(" ");
    Serial.print(vel);
    Serial.print(" ");
    Serial.print(dacOut);
    Serial.println();
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}

float velocityOut(float totalDistance, unsigned long totalTime, unsigned long currentTime) {
  float vMax = (totalDistance/(2*(totalTime/3)))*pow(10,6);
  float aMax = (totalDistance/(2*squaref(totalTime/3)))*pow(10,12);

  if (currentTime < (totalTime/3)) {
    vel = aMax*currentTime*pow(10,-6);
  }
  else if (currentTime > (totalTime/3) && currentTime < ((2*totalTime)/3)) {
    vel = vMax;
  }
  else if (currentTime > ((2*totalTime)/3) && currentTime < totalTime) {
    vel = vMax - aMax*(currentTime - (2*totalTime)/3)*pow(10,-6);
  }
  else {
    vel = 0;
  }

  return vel;
}

// For a given totalDistance and totalTime: vMax = totalDistance/(2*(totalTime/3)) & aMax = totalDistance/(2*((totalTime/3)^2))
float motionProfiling(float aMax, float vMax, float totalDistance, unsigned long currentTime) { 
    // totalDistance is total distance for motion profiling
    // currentTime is current time during motion profiling 

    // calculate the time it takes to accelerate to max veclocity 
    float accelTime = vMax/aMax;

    // halfDistance is the halfway distance between 0 and totalDistance
    // accelDistance is the distance you travel across the total duration of the acceleration 
    float halfDistance = totalDistance/2;
    float accelDistance = 0.5 * aMax * (accelTime * accelTime);

    // if accelDistance is greater than halfDistance the profile is impossible with that accelTime so recalculate accelTime
    // recalculated accelTime is the time it takes to travel halfDistance while accelerating at aMax (results in triangular velocity profile aka. no cruising)
    if (accelDistance > halfDistance) {
      accelTime = sqrt( halfDistance / (0.5 * aMax));   
    }
    
    // recalculate accelDistance with new accelTime
    accelDistance = 0.5 * aMax * (accelTime * accelTime);

    // recalculate max velocity based on the new time we have to accelerate and decelerate 
    vMax = aMax * accelTime;

    // cruiseDistance is the distance traveled while cruising
    // cruiseTime is the amount of time at cruising velocity
    float cruiseDistance = totalDistance - (2 * accelDistance);
    float cruiseTime = cruiseDistance / vMax;
    float deaccelTime = accelTime + cruiseTime;

    // check to see if we're still in the motion profile 
    float totalTime = 2 * (accelTime) + cruiseTime; 
    if (currentTime > totalTime){
      return totalDistance;
    }

    // if we're accelerating
    if (currentTime < accelTime) {
    // use the kinematic equation for acceleration
    return 0.5 * aMax * currentTime * currentTime;
    }

    // if we're cruising
    else if (currentTime < deaccelTime) {
    accelDistance = 0.5 * aMax * accelTime * accelTime;
    cruiseTime = currentTime - accelTime;

    // use the kinematic equation for constant velocity
    return accelDistance + vMax * cruiseTime;
  }

  // if we're decelerating
  else {
    accelDistance = 0.5 * aMax * accelTime * accelTime;
    cruiseDistance = vMax * cruiseTime;
    deaccelTime = currentTime - deaccelTime;

    // use the kinematic equations to calculate the instantaneous desired totalDistance
    return accelDistance + cruiseDistance + vMax * deaccelTime - 0.5 * aMax * deaccelTime * deaccelTime;
  }
}
