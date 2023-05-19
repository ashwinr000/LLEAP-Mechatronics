#define ChA 34
#define DAC 25
#define TPR 11
#define GR 300

long vel = 0;
int dacOut = 0;
String angle;
String t;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(DAC, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
    if (Serial.available() > 0) {
      String input = Serial.readString();
      int index = input.indexOf(' ');
      if (index != -1){
       angle = input.substring(0, index);
       t = input.substring(index+1);
      }
      trapezoid_profile(angle.toInt(), t.toInt());
  }
}

int dacOutput(float velocity) {
  float volts = 0.1 + (velocity - 0.0)*((5.0 - 0.1)/(5000.0-0.0)); // Convert from RPM to a Voltage Value
  int dacOut = 0 + (volts - 0.1)*((255 - 0)/(3.162-0.1)); // Covert from Voltage to DAC Output Value

  // Check if the DAC Output is Outside of its Acceptable Range
  if (dacOut < 0 || dacOut > 255) {
    dacOut = max(min(255, dacOut), 0);
  }

  return dacOut;
}

// Input a number of degrees travelled by the joint and a duration in seconds
void trapezoid_profile(int num_degs, unsigned long duration) {
  unsigned long t = 0;
  unsigned long tOffset = micros(); // Set the current time as the offset for the time
  while (t < duration*pow(10,6)) { // While the current time is less than the duration
    vel = velocityOut(num_degs*GR, duration*pow(10,6), t); // Calculate what the current velocity should be
    dacOut = dacOutput(vel); // Calculate the DAC Output of the current velocity
    t = micros() - tOffset;

    dacWrite(DAC, dacOut);

    Serial.print(t*pow(10,-6));
    Serial.print(" ");
    Serial.print(t);
    Serial.print(" ");
    Serial.print(vel);
    Serial.print(" ");
    Serial.print(dacOut);
    Serial.println();
  }

  dacWrite(DAC, 0);
}

float velocityOut(float totalDistance, unsigned long totalTime, unsigned long currentTime) {
  float vMax = ((totalDistance)/(2*(totalTime/3)))*pow(10,6); //Calculate Max Velocity from the total distance and total time, assuming equal time accellerating, coasting, and deccellerating
  float aMax = ((totalDistance)/(2*pow((totalTime/3),2)))*pow(10,12); //Calculate Max Accelleration using the same method as above

  if (currentTime < (totalTime/3)) {
    vel = aMax*currentTime*pow(10,-6); // While accelerating calculate the current velocity from time and acceleration
  }
  else if (currentTime > (totalTime/3) && currentTime < ((2*totalTime)/3)) {
    vel = vMax; // While coasting the current velocity is the max velocity
  }
  else if (currentTime > ((2*totalTime)/3) && currentTime < totalTime) {
    vel = vMax - aMax*(currentTime - (2*totalTime)/3)*pow(10,-6);  // While deccelerating calculate the current from time, acceleration, and the coasting velocity
  }
  else {
    vel = 0; // If the duration is over the current velocity should be zero
  }

  return vel/6; // Return current velocity in RPM
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