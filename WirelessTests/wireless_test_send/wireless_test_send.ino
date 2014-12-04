/*
* Simple Transmitter Code
* This code simply counts up to 255
* over and over
* (TX out of Arduino is Digital Pin 1)
*/
byte counter;

void setup(){
//2400 baud for the 434 model
Serial.begin(2400);
  counter = 0;
}
void loop(){
//send out to transmitter
  Serial.print(counter);
  counter++;
  delay(10);
}

