int pt1[3] = {A0, A1, A2};

void setup() {
  // put your setup code here, to run once:
  for(int i = 0; i < 3; i++) {
    pinMode(pt1[i], INPUT);
  }
  Serial.begin(4800);
}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i = 0; i < 3; i++) {
    Serial.print(analogRead(pt1[i]));
    Serial.print("\t");
  }
  Serial.print("\n");
  delay(500);
}
