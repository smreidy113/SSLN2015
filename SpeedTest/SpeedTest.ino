#include <digitalWriteFast.h>

int input = 22;

void setup() {
  // put your setup code here, to run once:
  pinMode(input, OUTPUT);
  digitalWrite(input, LOW);
  pinMode(input, INPUT);
  
  Serial.begin(9600);
}

char prevVal = 0;
unsigned long prevMicros = 0;

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalReadFast(input) != prevVal) {
    Serial.println(((micros() - prevMicros)));
    prevVal = digitalReadFast(input);
    prevMicros = micros();
  }
}
