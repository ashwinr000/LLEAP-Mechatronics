#define ENA 33
#define DAC 25
#define ChA 34

#define DPR 360.0 // Degrees rotated by joint per revolution of motor: 360 / (Revolutions of Motor / 1 Revolution of Joint)
#define TPR 11.0 // Hall Effect Sensor Ticks per Revolution of motor

#define maxVel 4800 //Maximum Velocity of Motor (degrees/second)

#define ERROR_MARGIN 0
#define Kp 1
#define Ki 0
#define Kd 0

int tickCount = 0;

float DT = 0;
float DPT = DPR / TPR;

unsigned long currentTime = 0;
unsigned long prevTime = 0;
unsigned long PIDOffset = 0;
unsigned long PIDTime = 0;

void setup() {
  // code runs once:
  Serial.begin(115200); //Start Serial Communication Rate at This Value
  Serial.print(" ");

  pinMode(hallSensor, INPUT); //Take Input from the Hall Sensor
  pinMode(DAC, OUTPUT); //Push a DAC Output to Motor Speed Controller
  pinMode(enable, OUTPUT); //Push an Enable Signal Output to Motor Controller
  pinMode(direction, OUTPUT); //Push a Direction Signal Output to Motor Controller
  digitalWrite(enable, HIGH);
  digitalWrite(direction, HIGH);
  currentTime = micros();
  prevTime = micros();

  digitalWrite(ENA, LOW);
  //dacWrite(DAC, 0);

  PID(162, 540, 3600); // 162 degs/sec^2, 540 degs/sec, 3600 degs
  
}

void loop() {
  // put your main code here, to run repeatedly:
   
}

void PID(float maxAccel, float maxSPD, float degs) {
  tickCount = 0;
  int sumError = 0;
  int prevError = degs - (DPT * tickCount); // Set previous error
  
  PIDOffset = micros();
  while ((degs - (DPT * tickCount)) > ERROR_MARGIN) {  // while the actual degrees minus the target degrees is greater than the error margin
    // Use motion profiling to generate an instantaneos target position
    PIDTime = micros() - PIDOffset;
    float instantTargetPosition = motionProfiling(maxAccel*pow(10,-12), maxSPD*pow(10,-6), degs, PIDTime); // maxAccel*10^-12 to convert from sec^2 to micros^2, maxSPD*pow(10,-6) to convert from sec to micros
    
    // Proportional Error Correction
    int error = instantTargetPosition - (DPT * tickCount); // Set current error to the difference between instant target degrees and actual degrees
    double P = Kp * error;
    
    // Integral Error Correction
    sumError += error * DT; // Integral of error with respect to currentTime
    double I = Ki * sumError;
    
    // Derivative Error Correction
    int changeError = (error - prevError) / DT; // Derivative of error with respect to currentTime
    double D = Kd * changeError;
    
    long vel = P + I + D;
    Serial.println(vel);
    
    //dacWrite(DAC, map(vel, 0, maxVel, 0, 255));
    prevError = error;
  }
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


 
void tickCounter() {
  tickCount++;
  prevTime = currentTime;
  currentTime = micros();
  DT = currentTime - prevTime;
}
