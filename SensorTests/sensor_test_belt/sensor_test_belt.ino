#include <VirtualWire.h>
#include <VirtualWire_Config.h>



int poutput = 13;

void pulseOut(int pin, int us)
{
   digitalWrite(pin, HIGH);
   us = max(us - 20, 1);
   delayMicroseconds(us);
   digitalWrite(pin, LOW);
}

void setup() {
  Serial.begin(4800);	// Debugging only
  Serial.println("setup");
  vw_set_ptt_pin(5);
  // Initialise the IO and ISR
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_setup(2400);	 // Bits per sec

  vw_rx_start();       // Start the receiver PLL running
  pinMode(poutput, OUTPUT);
  digitalWrite(poutput, LOW);
  //pinMode(6, OUTPUT);
}

void loop() {
  
  //digitalWrite(pinput1, HIGH);
  //delay(500);
  //digitalWrite(pinput1, LOW);
  //delay(500);
  
    uint8_t buf[VW_MAX_MESSAGE_LEN];
    uint8_t buflen = VW_MAX_MESSAGE_LEN;

    if (vw_get_message(buf, &buflen)) // Non-blocking
    {
        //digitalWrite(6, !digitalRead(6));
        //pulseOut(poutput, 10);
        digitalWrite(13, true); // Flash a light to show received good message
	// Message with a good checksum received, dump it.
	Serial.print("received");
	Serial.println(*buf);
        digitalWrite(13, false);
    }
}
