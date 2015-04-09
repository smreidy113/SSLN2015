#include <VirtualWire.h>
#include <VirtualWire_Config.h>

void setup() {
  vw_set_ptt_inverted(true);
  vw_setup(2400);
  
  vw_rx_start();
  Serial.begin(4800);
}

void loop() {
  uint8_t buf[VW_MAX_MESSAGE_LEN];
  uint8_t buflen = VW_MAX_MESSAGE_LEN;
  
  if (vw_get_message(buf, &buflen)) {
    if (buflen > 1) {
      for (int i = 0; i < buflen; i++) {
        Serial.println((char) buf[i]);
      }
    }
  }
}
