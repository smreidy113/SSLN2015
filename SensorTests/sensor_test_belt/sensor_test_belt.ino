int poutput = 13;

void pulseOut(int pin, int us)
{
   digitalWrite(pin, HIGH);
   us = max(us - 20, 1);
   delayMicroseconds(us);
   digitalWrite(pin, LOW);
}

void setup() {
  Serial.begin(9600);
  pinMode(poutput, OUTPUT);
  digitalWrite(output, LOW);
}

void loop() {
  pulseOut(poutput, 10);
  //digitalWrite(pinput1, HIGH);
  delay(500);
  //digitalWrite(pinput1, LOW);
  //delay(500);
}
