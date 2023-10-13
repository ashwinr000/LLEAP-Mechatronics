#define ENA 7
#define PWM 9
#define ChA 2

#define DPR 360.0 // Degrees rotated by joint per revolution of motor: 360 / (Revolutions of Motor / 1 Revolution of Joint)
#define TPR 11.0 // Hall Effect Sensor Ticks per Revolution of motor

#define minSPD 5
#define maxSPD 20

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
  turn(3600);

}

void loop() {
  // put your main code here, to run repeatedly:
   
}

void turn(int degs) {
   tickCount = 0;
   int sumError = 0;
   int prevError = degs - (DPT * tickCount); // Set previous error
   while (abs((DPT * tickCount) - degs) > ERROR_MARGIN) {  // while the actual degrees minus the target degrees is greater than the error margin
     int error = degs - (DPT * tickCount); // Set current error to the difference between target degrees and actual degrees
     double P = Kp * error;
     sumError += error * DT; // Integral of error with respect to time
     double I = Ki * sumError;
     int changeError = (error - prevError) / DT; // Derivative of error with respect to time
     double D = Kd * changeError;
     int vel = P + I + D;
     
     digitalWrite(ENA, HIGH);
     analogWrite(PWM, min(abs(vel), maxSPD));
     prevError = error;
   }
  
  analogWrite(PWM, 0);
  delay(1000);
  digitalWrite(ENA, LOW);
  Serial.println(tickCount);
  Serial.println(tickCount / TPR);
}

 
void tickCounter() {
  tickCount++;
  prevTime = time;
  time = micros();
  DT = time - prevTime;
}
