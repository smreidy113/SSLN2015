int pinput1 = 2;
int pinput2 = 3;
int poutput = 13;
unsigned long duration1;
unsigned long duration2;

void pulseOut(int pin, int us)
{
   digitalWrite(pin, HIGH);
   us = max(us - 20, 1);
   delayMicroseconds(us);
   digitalWrite(pin, LOW);
}

void setup() {
  Serial.begin(9600);
  pinMode(pinput1, INPUT);
  pinMode(pinput2, INPUT);
  digitalWrite(pinput1, LOW);
  digitalWrite(pinput2, LOW);
}

void loop() {
  pinMode(pinput1, OUTPUT);
  pulseOut(pinput1, 0);
  pinMode(pinput1, INPUT);
  digitalWrite(pinput1, LOW);
  duration = pulseIn(pinput1, HIGH)/29;
  pinMode(pinput2, OUTPUT);
  pulseOut(pinput2, 0);
  pinMode(pinput2, INPUT);
  digitalWrite(pinput2, LOW);
  duration1 = pulseIn(pinput2, HIGH)/29;
  Serial.print(duration);
  Serial.print("\n");
  //digitalWrite(pinput1, HIGH);
  delay(500);
  //digitalWrite(pinput1, LOW);
  //delay(500);
}
