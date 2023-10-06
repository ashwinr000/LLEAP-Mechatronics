#define ChA 34
#define DAC 25
#define TPR 11
#define EN 12
#define DIR 14

#define maxSpd 255 //analogWrite value corresponding to 5V which is ~2500 RPM

unsigned long currentTime = 0;
unsigned long prevTime = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  pinMode(ChA, INPUT);
  pinMode(DAC, OUTPUT);
  pinMode(EN, INPUT);
  pinMode(DIR, INPUT);
  digitalWrite(EN, HIGH);
  digitalWrite(DIR, HIGH);
  attachInterrupt(digitalPinToInterrupt(ChA), tickCounter, RISING); //run tickCounter everytime the Hall Effect sensor triggers high
}

void loop() {
  //put your main code here, to run repeatedly:
  
  delay(5000);
  analogWrite(DAC, 0);  //run at 0 RPM
  Serial.println(0);
  delay(5000);
  analogWrite(DAC, (maxSpd * 4) / 100); //run at 10% max speed ~250 RPM
  Serial.println(10);
  delay(5000);
  analogWrite(DAC, (maxSpd * 8) / 100); //run at 20% max speed ~500 RPM
  Serial.println(20);
  delay(5000);
  analogWrite(DAC, (maxSpd * 12) / 100); //run at 20% max speed ~750 RPM
  Serial.println(30);
  delay(5000);
  analogWrite(DAC, (maxSpd * 16) / 100); //run at 40% max speed ~1000 RPM
  Serial.println(40);
  delay(5000);
  analogWrite(DAC, (maxSpd * 20) / 100); //run at 50% max speed ~1250 RPM
  Serial.println(50);
  delay(5000);
  analogWrite(DAC, (maxSpd * 24) / 100); //run at 60% max speed ~1500 RPM
  Serial.println(60);
  delay(5000);
  analogWrite(DAC, (maxSpd * 28) / 100); //run at 70% max speed ~1750 RPM
  Serial.println(70);
  delay(5000);
  analogWrite(DAC, (maxSpd * 32) / 100); //run at 80% max speed ~2000 RPM
  Serial.println(80);
  delay(5000);
  analogWrite(DAC, (maxSpd * 36) / 100); //run at 90% max speed ~2250 RPM
  Serial.println(90);
  delay(5000);
  analogWrite(DAC, min(((maxSpd * 40) / 100), 255)); //run at 100% max speed ~2500 RPM
  Serial.println(100);
  delay(5000);
  analogWrite(DAC, 0);
}

void tickCounter() {
  prevTime = currentTime; // record new time as old time
  currentTime = micros(); //update new time
  Serial.println((60000000 / (currentTime - prevTime)) / TPR); // calculate RPM using difference between old and new time and ticks per revolution
}
