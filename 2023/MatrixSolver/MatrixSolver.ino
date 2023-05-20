#define ChA 34
#define DAC 25
#define TPR 11
#define GR 300

float velocity = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    String input = Serial.readString();
    int index = input.indexOf(' ');
    String x1_s = "";
    String v1_s = "";
    String x2_s = "";
    String v2_s = "";
    String duration_s = "";

    if (index != -1){ 
      x1_s = input.substring(0, index);
      
      if (index != -1){    
        input = input.substring(index+1);
        index = input.indexOf(' ');
        v1_s = input.substring(0, index);
        
        if (index != -1){    
          input = input.substring(index+1);
          index = input.indexOf(' ');
          x2_s = input.substring(0, index);
          
          if (index != -1){    
            input = input.substring(index+1);
            index = input.indexOf(' ');
            v2_s = input.substring(0, index);
            
            if (index != -1){    
              input = input.substring(index+1);
              index = input.indexOf(' ');
              duration_s = input.substring(0, index);
            }
          }
        }
      }
    }
    float x1 = x1_s.toFloat();
    float v1 = v1_s.toFloat();
    float x2 = x2_s.toFloat();
    float v2 = v2_s.toFloat();
    float duration = duration_s.toFloat();
    splines(x1,v1,x2,v2,duration);
  }
} 

// Input 2 position and velocities of the joint and the duration of the move
void splines(float x1, float v1, float x2, float v2, float duration) {
  float t = 0;
  float tStart = micros()*pow(10,-6);
  while (t < duration) { // While the current time is less than the end time
    int dacOut = MatrixSolver(GR*x1, GR*v1, 0, GR*x2, GR*v2, duration, t); // Calculate what the current velocity should be
    t = (micros()*pow(10,-6))-tStart;

    dacWrite(DAC, dacOut);

    Serial.print(t);
    Serial.print(" ");
    Serial.print(velocity);
    Serial.print(" ");
    Serial.print(dacOut);
    Serial.println();
  }
  dacWrite(DAC, 0);
}

int MatrixSolver(float x1, float v1, float t1, float x2, float v2, float t2, float tnow) {
  // 1st Equation/Row
  float a = pow(t1,3);
  float b = pow(t1,2);
  float c = t1;
  float d = 1;
  float q = x1;

  // 2nd Equation/Row
  float e = pow(t2,3);
  float f = pow(t2,2);
  float g = t2;
  float h = 1;
  float r = x2;

  // 3rd Equation/Row
  float i = 3*pow(t1,2);
  float j = 2*t1;
  float k = 1;
  float l = 0;
  float s = v1;

  // 4th Equation/Row
  float m = 3*pow(t2,2);
  float n = 2*t2;
  float o = 1;
  float p = 0;
  float t = v2;

  float r1 = -b*g*l*t+b*g*p*s+b*h*k*t-b*h*o*s-b*k*p*r+b*l*o*r+c*f*l*t-c*f*p*s-c*h*j*t+c*h*n*s+c*j*p*r-c*l*n*r-d*f*k*t+d*f*o*s+d*j*g*t-d*g*n*s-d*j*o*r+d*k*n*r+f*k*p*q-f*l*o*q-g*j*p*q+g*l*n*q+h*j*o*q-h*k*n*q;
  float r2 = a*g*l*t-a*g*p*s-a*h*k*t+a*h*o*s+a*k*p*r-a*l*o*r-c*e*l*t+c*e*p*s+c*h*i*t-c*h*m*s-c*i*p*r+c*l*m*r+d*e*k*t-d*e*o*s-d*g*i*t+d*g*m*s+d*i*o*r-d*k*m*r-e*k*p*q+e*l*o*q+g*i*p*q-g*l*m*q-h*i*o*q+h*k*m*q;
  float r3 = -a*f*l*t+a*f*p*s+a*h*j*t-a*h*n*s-a*j*p*r+a*l*n*r+b*e*l*t-b*e*p*s-b*h*i*t+b*h*m*s+b*i*p*r-b*l*m*r-d*e*j*t+d*e*n*s+d*f*i*t-d*f*m*s-d*i*n*r+d*j*m*r+e*j*p*q-e*l*n*q-f*i*p*q+f*l*m*q+h*i*n*q-h*j*m*q;
  float r4 = a*f*k*t-a*f*o*s-a*g*j*t+a*g*n*s+a*j*o*r-a*k*n*r-b*e*k*t+b*e*o*s+b*g*i*t-b*g*m*s-b*i*o*r+b*k*m*r+c*e*j*t-c*e*n*s-c*f*i*t+c*f*m*s+c*i*n*r-c*j*m*r-e*j*o*q+e*k*n*q+f*i*o*q-f*k*m*q-g*i*n*q+g*j*m*q;
  float den = a*f*k*p-a*f*l*o-a*g*j*p+a*g*l*n+a*h*j*o-a*h*k*n-b*e*k*p+b*e*l*o+b*g*i*p-b*g*l*m-b*h*i*o+b*h*k*m+c*e*j*p-c*e*l*n-c*f*i*p+c*f*l*m+c*h*i*n-c*h*j*m-d*e*j*o+d*e*k*n+d*f*i*o-d*f*k*m-d*g*i*n+d*g*j*m;

  float c1 = r1/den;
  float c2 = r2/den;
  float c3 = r3/den;
  float c4 = r4/den;

  float vel = 3*c1*pow(tnow,2) + 2*c2*tnow + c3; // Current Velocity in degs/s
  velocity = vel/60; // Convert from degs/s to RPM

  float volts = 0.1 + (velocity - 0.0)*((5.0 - 0.1)/(5000.0-0.0)); // Convert from RPM to a Voltage Value
  int dacOut = 0 + (volts - 0.1)*((255 - 0)/(3.162-0.1)); // Covert from Voltage to DAC Output Value

  // Check if the DAC Output is Outside of its Acceptable Range
  if (dacOut < 0 || dacOut > 255) {
    dacOut = max(min(255, dacOut), 0);
  }
  return dacOut;
}