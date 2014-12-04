#define DIST_BETWEEN_SENSORS 6.4

#include <VirtualWire.h>
#include <VirtualWire_Config.h>
#include <stdlib.h>

double getLocInfo(int d1, int d2) {
  double beta = acos((pow(d2,2)+pow(DIST_BETWEEN_SENSORS,2)-pow(d1,2))/(2*d2*DIST_BETWEEN_SENSORS));
  double d = sqrt(pow(sin(beta),2)*pow(d2,2)+pow(DIST_BETWEEN_SENSORS/2-cos(beta)*d2,2));
  return d;
}

double getOrientationInfo(double d, int d1, int d2) {
  double beta = acos((pow(d2,2)+pow(DIST_BETWEEN_SENSORS,2)-pow(d1,2))/(2*d2*DIST_BETWEEN_SENSORS));
  double theta = 3.141592/2 - asin(sin(beta)*d2/d);
  return theta;
}

int pinput1 = 2;
int pinput2 = 3;
int poutput = 13;
unsigned long duration;
unsigned long duration1;

void pulseOut(int pin, int us)
{
   digitalWrite(pin, HIGH);
   us = max(us - 20, 1);
   delayMicroseconds(us);
   digitalWrite(pin, LOW);
}

void setup() {
  Serial.begin(4800);	  // Debugging only

  // Initialise the IO and ISR
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_setup(2400);	 // Bits per sec
  pinMode(pinput1, INPUT);
  pinMode(pinput2, INPUT);
  digitalWrite(pinput1, LOW);
  digitalWrite(pinput2, LOW);
}

int getTime(int pin) {
  pinMode(pin, OUTPUT);
  pulseOut(pin, 0);
  pinMode(pin, INPUT);
  digitalWrite(pin, LOW);
  return pulseIn(pin, HIGH)/29;
}



void loop() {
  const char *msg = "p";
  digitalWrite(13, HIGH);
  vw_send((uint8_t *)msg, strlen(msg));
  //vw_wait_tx(); // Wait until the whole message is gone
  digitalWrite(13, LOW);
  duration = getTime(pinput1);
  vw_send((uint8_t *)msg, strlen(msg));
  duration1 = getTime(pinput2);
  double dist = getLocInfo(duration, duration1);
  double ang = getOrientationInfo(dist, duration, duration1);
  Serial.print(duration);
  Serial.print("\t");
  Serial.print(duration1);
  Serial.print("\tTotal Distance: ");
  Serial.print(dist);
  Serial.print("\tOrientation: ");
  Serial.print(ang);
  Serial.print("\n");
}
