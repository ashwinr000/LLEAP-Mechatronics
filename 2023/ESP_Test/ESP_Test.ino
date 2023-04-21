int counter = 0;

void setup() {
  Serial.begin(115200);
  //pinMode(25, INPUT);
  //attachInterrupt(digitalPinToInterrupt(25), counterTick, RISING);

}

void loop() {
  //Serial.println(counter);
}

void counterTick() {
  counter++;
}