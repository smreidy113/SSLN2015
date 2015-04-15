int analogPins[5] = {A0, A1, A2, A3, A4};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i = 0; i < 5; i++) {
    Serial.print(analogRead(analogPins[i]));
    Serial.print("\t");
  }
  Serial.print("\n");
}
