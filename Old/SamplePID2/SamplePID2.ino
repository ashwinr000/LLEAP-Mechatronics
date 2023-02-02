

#define INA1 7
#define INA2 8
#define PWM 9
#define ChA 3

#define ERROR_MARGIN 0 
#define DT 0.000000001
#define Kp 1.5
#define Ki 0
#define Kd 0

int tickCount = 0;
int spd = 0;
int posT = 360;

int fromLow = 0;
int fromHigh = 255;
int toLow = 20;
int toHigh = 255;

unsigned long startTime = 0;
unsigned long endTime = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
 
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode (ChA, INPUT);
  attachInterrupt(digitalPinToInterrupt(ChA), tickCounter, RISING);
  startTime = millis();
  digitalWrite(INA1, HIGH);
  digitalWrite(INA2, LOW);
  //analogWrite(PWM, 255);
  turn(360); 
  
}

void loop() {
  // put your main code here, to run repeatedly:
   
   
}

void turn(int degs) {
   tickCount = 0;
   int sumError = 0;
   int prevError = degs - tickCount;
   while (abs(tickCount - degs) > ERROR_MARGIN) {
     int error = degs - tickCount;
     double P = Kp * error;
     sumError += error * DT;
     double I = Ki * sumError;
     int changeError = (error - prevError) / DT;
     double D = Kd * changeError;
     int vel = (int)(P + I + D);

    int nVel = (int)map(vel, fromLow, fromHigh, toLow, toHigh);
     
     analogWrite(PWM, min(nVel, 255));
    // delay(DT * 1000);
     prevError = error;
     Serial.print(tickCount);
     Serial.print(" ");
     Serial.print(P);
     Serial.print(" ");
     Serial.print(I);
     Serial.print(" ");
     Serial.print(D);
     Serial.print(" ");
     Serial.println(nVel);
   }
   
   analogWrite(PWM,0);
}

 
void tickCounter() {
  tickCount++;
//  if (tickCount == posT) {
//    endTime = millis();
//    Serial.println(endTime - startTime);
//    analogWrite(PWM, 0);
//    }
}
