#define ENA 7
#define PWM 9
#define ChA 2

#define DPR 360.0 // Degrees rotated by joint per revolution of motor: 360 / (Revolutions of Motor / 1 Revolution of Joint)
#define TPR 11.0 // Hall Effect Sensor Ticks per Revolution of motor

#define maxVel 2500 //Maximum Velocity of Motor (RPM)

#define ERROR_MARGIN 0
#define Kp 1
#define Ki 0
#define Kd 0

int tickCount = 0;

float DT = 0;
float DPT = DPR / TPR;

unsigned long time = 0;
unsigned long prevTime = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
 
  pinMode(ENA, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode (ChA, INPUT);
  attachInterrupt(digitalPinToInterrupt(ChA), tickCounter, RISING); //run tickCounter everytime the Hall Effect sensor triggers high
  time = micros();
  prevTime = micros();

  digitalWrite(ENA, LOW);
  analogWrite(PWM, 0);

}

void loop() {
  // put your main code here, to run repeatedly:
   
}

void PID(float maxAccel, float maxSPD, float degs) {
   tickCount = 0;
   int sumError = 0;
   int prevError = degs - (DPT * tickCount); // Set previous error
   while ((degs - (DPT * tickCount)) > ERROR_MARGIN) {  // while the actual degrees minus the target degrees is greater than the error margin
      float instantTargetPosition = motionProfiling(maxAccel, maxSPD, degs, time);
      int error = instantTargetPosition - (DPT * tickCount); // Set current error to the difference between instant target degrees and actual degrees
      double P = Kp * error;
      sumError += error * DT; // Integral of error with respect to time
      double I = Ki * sumError;
      int changeError = (error - prevError) / DT; // Derivative of error with respect to time
      double D = Kd * changeError;
      long vel = P + I + D;

      analogWrite(PWM, map(vel, 0, maxVel, 0, 255));
      prevError = error;
   }
}

float motionProfiling(float aMax, float vMax, float x, unsigned long dt_mp) { 
    //x is distance for motion profiling  
    //dt_mp is change in time for motion profiling 

    //calculate the time it takes to accelerate to max veclocity 
    float accel_dt = vMax/aMax;

    // half_x is the length of the distance that goes from 0 to max vecloity at max acceleration
    // accel_x is the distance you travel across the total duration of your acceleration 
    float half_x = x/2;
    float accel_x = 0.5 * aMax * (accel_dt * accel_dt);

    if (accel_x > half_x) {
      accel_dt = sqrt( half_x / (0.5 * aMax));   
    }
      

    accel_x = 0.5 * aMax * (accel_dt * accel_dt);

    // recalculate max velocity based on the new time we have to accelerate and decelerate 
    vMax = aMax * accel_dt;

    // calculate the time that you're at constant max velocity (cruising distance)
    //cruise_dt is the amount of time it takes to cruise 
    float cruise_x = x - (2 * accel_x);
    float cruise_dt = cruise_x / vMax;
    float deaccel_time = accel_dt + cruise_dt;

    //check to see if we're still in the motion profile 
    float entire_dt = 2 * (accel_dt) + cruise_dt; 
    if (dt_mp > entire_dt){
      return x;
    }

    // if we're accelerating
    if (dt_mp < accel_dt) {
    // use the kinematic equation for acceleration
    return 0.5 * aMax * dt_mp * dt_mp;
    }

    // if we're cruising
    else if (dt_mp < deaccel_time) {
    accel_dt = 0.5 * aMax * accel_dt * accel_dt;
    float cruise_dt_mp = dt_mp - accel_dt;

    // use the kinematic equation for constant velocity
    return accel_x + vMax * cruise_dt_mp;
  }

  // if we're decelerating
  else {
    accel_x = 0.5 * aMax * accel_dt * accel_dt;
    cruise_x = vMax * cruise_dt;
    deaccel_time = dt_mp - deaccel_time;

    // use the kinematic equations to calculate the instantaneous desired position
    return accel_x + cruise_x + vMax * deaccel_time- 0.5 * aMax * deaccel_time * deaccel_time;
  }





  }


 
void tickCounter() {
  tickCount++;
  prevTime = time;
  time = micros();
  DT = time - prevTime;
}
