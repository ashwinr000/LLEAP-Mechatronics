int counter = 0;
int dacOut = 0;
String angle;
String t;

void setup() {
  Serial.begin(115200);
  //pinMode(25, INPUT);
  //attachInterrupt(digitalPinToInterrupt(25), counterTick, RISING);

  /*dacOut = mapping(250);
  dacWrite(25, dacOut);
  Serial.print("DAC ");
  Serial.print(dacOut);
  Serial.println();*/

}

void loop() {

  if (Serial.available() > 0) {
      String input = Serial.readString();
      int index = input.indexOf(' ');
      if (index != -1){
       angle = input.substring(0, index);
       t = input.substring(index+1);
      }
      Serial.println(angle);
      Serial.println(t);
  }
}

int mapping(float velocity) {
  Serial.print("RPM ");
  Serial.print(velocity);
  Serial.println();
  float volts = 0.1 + (velocity - 0.0)*((5.0 - 0.1)/(5000.0-0.0));
  Serial.print("Volts ");
  Serial.print(volts);
  Serial.println();
  int dacOut = 0 + (volts - 0.1)*((255 - 0)/(3.162-0.1));

  if (dacOut < 0 || dacOut > 255) {
    dacOut = max(min(255, dacOut), 0);
  }

  return dacOut;
}

void counterTick() {
  counter++;
}