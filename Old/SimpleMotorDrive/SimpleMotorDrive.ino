
#define INA1 7
#define INA2 8
#define PWM 9

void setup() {
  // put your setup code here, to run once:
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(PWM, OUTPUT);

  digitalWrite(INA1, HIGH);
  digitalWrite(INA2, LOW);
  analogWrite(PWM, 127);
  delay(10000);
  analogWrite(PWM, 0);

}

void loop() {
  // put your main code here, to run repeatedly:

}
