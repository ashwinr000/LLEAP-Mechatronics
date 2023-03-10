#define Dt 5

float target = 0;
unsigned long t = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  while (target < 3600) {
    target = motionProfiling(0.000144, 0.720, 3600, t);
    Serial.println(target);
    delay(Dt);
    t = t + Dt;
  }
}

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
