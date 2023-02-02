

#define INA1 7
#define INA2 8
#define PWM 9
#define ChA 3
int tickCount = 0;
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
  analogWrite(PWM, 127);
 
  
  
}

void loop() {
  // put your main code here, to run repeatedly:
 
}

void tickCounter() {
  tickCount++;
  if (tickCount == 360) {
    endTime = millis();
    Serial.println(endTime - startTime);
  analogWrite(PWM, 0);
  }
}
