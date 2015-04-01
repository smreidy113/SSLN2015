int pwm1 = 6;
int dir1 = 52;
int pwm2 = 5;
int dir2 = 53;

int l = 13;

void setup() {
  // put your setup code here, to run once:
  pinMode(pwm1, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(l, OUTPUT);
  digitalWrite(l, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(dir1,LOW);
  digitalWrite(dir2,HIGH);
  analogWrite(pwm1,35);
  analogWrite(pwm2,35);
}
