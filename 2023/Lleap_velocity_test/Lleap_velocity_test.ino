#define ChA 2
#define PWM 9
#define TPR 11

int maxSpd = 255; //analogWrite value corresponding to 5V which is ~2500 RPM

unsigned long time = 0;
unsigned long prevTime = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  pinMode(ChA, INPUT);
  pinMode(PWM, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ChA), tickCounter, RISING); //run tickCounter everytime the Hall Effect sensor triggers high
}

void loop() {
  //put your main code here, to run repeatedly:
  
  delay(5000);
  analogWrite(PWM, 0);  //run at 0 RPM
  Serial.println(0);
  delay(5000);
  analogWrite(PWM, (maxSpd * 10) / 100); //run at 10% max speed ~250 RPM
  Serial.println(10);
  delay(5000);
  analogWrite(PWM, (maxSpd * 20) / 100); //run at 20% max speed ~500 RPM
  Serial.println(20);
  delay(5000);
  analogWrite(PWM, (maxSpd * 30) / 100); //run at 20% max speed ~750 RPM
  Serial.println(30);
  delay(5000);
  analogWrite(PWM, (maxSpd * 40) / 100); //run at 40% max speed ~1000 RPM
  Serial.println(40);
  delay(5000);
  analogWrite(PWM, (maxSpd * 50) / 100); //run at 50% max speed ~1250 RPM
  Serial.println(50);
  delay(5000);
  analogWrite(PWM, (maxSpd * 60) / 100); //run at 60% max speed ~1500 RPM
  Serial.println(60);
  delay(5000);
  analogWrite(PWM, (maxSpd * 70) / 100); //run at 70% max speed ~1750 RPM
  Serial.println(70);
  delay(5000);
  analogWrite(PWM, (maxSpd * 80) / 100); //run at 80% max speed ~2000 RPM
  Serial.println(80);
  delay(5000);
  analogWrite(PWM, (maxSpd * 90) / 100); //run at 90% max speed ~2250 RPM
  Serial.println(90);
  delay(5000);
  analogWrite(PWM, min(((maxSpd * 100) / 100), 255)); //run at 100% max speed ~2500 RPM
  Serial.println(100);
  delay(5000);
  analogWrite(PWM, 0);
}

void tickCounter() {
  prevTime = time; // record new time as old time
  time = micros(); //update new time
  Serial.println((60000000 / (time - prevTime)) / TPR); // calculate RPM using difference between old and new time and ticks per revolution
}
