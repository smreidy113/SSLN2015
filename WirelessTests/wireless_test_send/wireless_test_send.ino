#include <VirtualWire.h>
#include <VirtualWire_Config.h>

/*
* Simple Transmitter Code
* This code simply counts up to 255
* over and over
* (TX out of Arduino is Digital Pin 1)
*/


byte counter;
int pin = 13;

char *controller;

void setup(){
  pinMode(1,OUTPUT);
  //2400 baud for the 434 model
  vw_set_ptt_inverted(true);
  //vw_set_tx_pin(12);
  vw_setup(2400);
  counter = 0;
  Serial.begin(4800);
  pinMode(13,OUTPUT);
}
void loop(){
//send out to transmitter
  controller="1"  ;
  vw_send((uint8_t *)controller, strlen(controller));
//vw_wait_tx(); // Wait until the whole message is gone
digitalWrite(13,1);
delay(100);
controller="0"  ;
  vw_send((uint8_t *)controller, strlen(controller));
//vw_wait_tx(); // Wait until the whole message is gone
digitalWrite(13,0);
delay(100);
}

