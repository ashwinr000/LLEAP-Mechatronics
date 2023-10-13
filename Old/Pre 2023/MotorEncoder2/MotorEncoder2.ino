

#define INA1 7
#define INA2 8
#define PWM 9
#define ChA 3
int tickCount = 0;
int spd = 0;
int posT = 360;
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
 // digitalWrite(INA1, HIGH);
 // digitalWrite(INA2, LOW);
 // analogWrite(PWM, 127);
   
  
}

void loop() {
  // put your main code here, to run repeatedly:
   
   
}

  float kp = 2;
  float deltaY = posT-tickCount;
  float u = kp*deltaY;

// Set the motor speed and direction
  int dir = 1;
  if (u<0){
    dir = -1;
  }
  int spd = (int) fabs(u);
  if(spd > 255){
    spd = 255;
  }
  setMotor(dir,spd,PWM,IN1,IN2);

void setMotor(int dir, int pwmVal, int PWM, int INA1, int IN2){
  analogWrite(PWM,pwmVAL); // Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);    
  }
}

void tickCounter() {
  tickCount++;
  if (tickCount == posT) {
    endTime = millis();
    Serial.println(endTime - startTime);
    analogWrite(PWM, 0);
    }
}
