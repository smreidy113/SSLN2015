int analogPins[14] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("\t");
  for (int i = 0; i < 14; i++) {
    Serial.print(analogRead(analogPins[i]));
    Serial.print("\t");
  }
  Serial.print("\n");
}
