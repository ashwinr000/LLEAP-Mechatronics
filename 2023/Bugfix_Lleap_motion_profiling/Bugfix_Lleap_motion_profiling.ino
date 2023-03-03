unsigned long tOffset = 0;
unsigned long t = 0;
unsigned long prevt = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  
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
