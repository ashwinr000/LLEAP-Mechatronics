#define hallSensor 34 //Hall Effect Sensor Pin
#define DAC 25 //Motor Speed Controller Pin
#define tickPerRev 11 //Hall-Effect Ticks per Revolution of Motor
#define enable 33 //Motor Enable Pin
#define direction 32 //Motor Direction Pin

#define maxSpd 255 //dacWrite value corresponding to 3.3V Output to Motor Controller which is a Motor Speed of ~ RPM

unsigned long currentTime = 0;
unsigned long prevTime = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); //Start Serial Communication Rate at this Value
  
  pinMode(hallSensor, INPUT); //Take Input from the Hall Sensor
  pinMode(DAC, OUTPUT); //Push a DAC Output to Motor Speed Controller
  pinMode(enable, OUTPUT); //Push an Enable Signal Output to Motor Controller
  pinMode(direction, OUTPUT); //Push a Direction Signal Output to Motor Controller
  digitalWrite(enable, HIGH);
  digitalWrite(direction, HIGH);
  
  //Run tickCounter Function Everytime the Hall Effect Sensor Triggers a High Input
  attachInterrupt(digitalPinToInterrupt(hallSensor), tickCounter, RISING);
}

void loop() {
  //put your main code here, to run repeatedly:
  
  delay(5000);
  dacWrite(DAC, 0);  //Run at 0 RPM
  Serial.println(0);
  delay(5000);
  dacWrite(DAC, (maxSpd * 4) / 100); //Run at 4% Max Speed
  Serial.println(4);
  delay(5000);
  dacWrite(DAC, (maxSpd * 8) / 100); //Run at 8% Max Speed
  Serial.println(8);
  delay(5000);
  dacWrite(DAC, (maxSpd * 12) / 100); //Run at 12% Max Speed
  Serial.println(12);
  delay(5000);
  dacWrite(DAC, (maxSpd * 16) / 100); //Run at 16% Max Speed
  Serial.println(16);
  delay(5000);
  dacWrite(DAC, (maxSpd * 20) / 100); //Run at 20% Max Speed
  Serial.println(20);
  delay(5000);
  dacWrite(DAC, (maxSpd * 24) / 100); //Run at 24% Max Speed
  Serial.println(24);
  delay(5000);
  dacWrite(DAC, (maxSpd * 28) / 100); //Run at 28% Max Speed
  Serial.println(28);
  delay(5000);
  dacWrite(DAC, (maxSpd * 32) / 100); //Run at 32% Max Speed
  Serial.println(32);
  delay(5000);
  dacWrite(DAC, (maxSpd * 36) / 100); //Run at 36% Max Speed
  Serial.println(36);
  delay(5000);
  dacWrite(DAC, (maxSpd * 40) / 100); //Run at 40% Max Speed
  Serial.println(40);
  delay(5000);
  dacWrite(DAC, 0);
}

void tickCounter() {
  prevTime = currentTime; // record new time as old time
  currentTime = micros(); //update new time

  // calculate RPM using difference between old and new time and ticks per revolution
  Serial.println((60000000 / (currentTime - prevTime)) / tickPerRev);
}
