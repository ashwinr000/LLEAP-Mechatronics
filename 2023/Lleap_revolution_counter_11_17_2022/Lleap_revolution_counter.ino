#define ChA 2
#define PWM 9
#define TPR 33

int maxSpd = 255; //analogWrite value corresponding to 5V
int tickCount = 0;

unsigned long time = 0;
unsigned long prevTime = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  pinMode(ChA, INPUT);
  pinMode(PWM, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ChA), tickCounter, RISING); //run tickCounter everytime the Hall Effect sensor triggers high
  analogWrite(PWM, 128);
}

void loop() {
  //put your main code here, to run repeatedly:

}

void tickCounter() {
  tickCount++;
  prevTime = time; // record new time as old time
  time = micros(); //update new time
  if (tickCount >= 100*TPR) { // stop motor after ~ 10 revolutions
    analogWrite(PWM, 0);
  }
  Serial.println((60000000 / (time - prevTime)) / 11); // calculate RPM using difference between old and new time and 11 ticks per revolution
}
