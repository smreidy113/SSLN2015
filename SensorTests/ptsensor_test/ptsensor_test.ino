int pin = A0;
int reading;

void setup() {
  pinMode(pin, INPUT);
  Serial.begin(4800);
}

void loop() {
  delay(50);
  reading = analogRead(pin);
  Serial.print(reading);
  Serial.print("\n");
}
