#define ChA 2
#define PWM 9
#define INA 7



#define ERROR_MARGIN 5 
#define DT 0.000000001
#define Kp 1.5
#define Ki 0
#define Kd 0
#define gearRatio 1 // 1 tick = 360 degrees, motor geared down by gearRatio

int tickCount = 0;

unsigned long newTime = 0;
unsigned long oldTime = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
 
  pinMode (ChA, INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(INA1, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ChA), tickCounter, RISING);
  oldTime = millis();
  newTime = millis();

  digitalWrite(INA, HIGH);
  turn(360);
  
}

void loop() {
  // put your main code here, to run repeatedly:
   
   
}

void turn(int degs) {
    tickCount = 0;
    int sumError = 0;
    int prevError = degs - tickCount;
    while (abs(((360*tickCount)/gearRatio) - degs) > ERROR_MARGIN) {
      int error = degs - ((360*tickCount)/gearRatio);
      double P = Kp * error;
      sumError += error * DT;
      double I = Ki * sumError;
      int changeError = (error - prevError) / DT;
      double D = Kd * changeError;
      int vel = (int)(P + I + D);
     
      analogWrite(PWM, min(vel, 255));
      prevError = error;
    // delay(DT * 1000);
    // Serial.print(tickCount);
    // Serial.print(" ");
    // Serial.print(P);
    // Serial.print(" ");
    // Serial.print(I);
    // Serial.print(" ");
    // Serial.print(D);
    // Serial.print(" ");
    // Serial.println(vel);
   }
   
   analogWrite(PWM,0);
}

 
void tickCounter() {
  tickCount++;
  oldTime = newTime
  newTime = millis()
  DT = newTime-oldTime
}