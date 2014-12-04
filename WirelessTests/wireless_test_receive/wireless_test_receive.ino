/*
* Simple Receiver Code
* (TX out of Arduino is Digital Pin 1)
* (RX into Arduino is Digital Pin 0)
*/
int incomingByte = 0;
int onoff = 0;
int pin = 13;
void setup(){
  //2400 baud for the 434 model
  Serial.begin(2400);
}
void loop(){
  // read in values, debug to compute
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    if (onoff == 0) {
      digitalWrite(pin, HIGH);
      onoff = 1;
    } else {
      digitalWrite(pin, LOW);
      onoff = 0;
    }
    Serial.println(incomingByte, DEC);
  }
  incomingByte = 0;
}
