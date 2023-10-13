#define ChA 34
#define DAC 25
#define TPR 11

#define maxSpd 255 // dacWrite value corresponding to maximum allowed speed

int tickCount = 0;

unsigned long currentTime = 0;
unsigned long prevTime = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  pinMode(ChA, INPUT);
  pinMode(DAC, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ChA), tickCounter, RISING); //run tickCounter everytime the Hall Effect sensor triggers high
  dacWrite(DAC, 128);
}

void loop() {
  //put your main code here, to run repeatedly:

}

void tickCounter() {
  tickCount++;
  prevTime = currentTime; // record new time as old time
  currentTime = micros(); //update new time
  if (tickCount >= 100*TPR) { // stop motor after ~ 10 revolutions
    dacWrite(DAC, 0);
  }
  Serial.println((60000000 / (currentTime - prevTime)) / 11); // calculate RPM using difference between old and new time and 11 ticks per revolution
}
