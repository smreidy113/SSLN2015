#include <digitalWriteFast.h>
#include <VirtualWire.h>
#include <VirtualWire_config.h>

#define X 26.0
#define Y 32.0
#define ZP 52.5
#define BETAD 0.1
#define BETAO 0.1

float Z = sqrt(pow(Y,2)+pow(ZP,2));
float t = atan2(ZP,Y);
float cost = cos(t);
float sint = sin(t);

float triang[2];
float ftriang[2];

float prevDist;
float prevOrientation;

char justStartedD = 1;
char justStartedO = 1;

void *getLocInfo(int l1, int l3, int l2) {
  //float info[2];
  float d3 = (pow(l1,2)-pow(l2,2))/(2*X);
  //Serial.print(d3);
  //Serial.print("\t");
  float d1 = (pow(l2,2)-pow(l3,2)+pow(Z,2)-1/4.0*pow(X,2)+d3*X)/(2*Z);
  //Serial.print(d1);
  //Serial.print("\t");
  float d2 = sqrt(pow(l3,2)-pow(d3,2)-pow((d1-Z),2));
  //Serial.print(d2);
  //Serial.print("\t");
  float dp1 = cost*d1 + sint*d2;
  float dp3 = d3;
  float dist = sqrt(pow(dp1,2)+pow(dp3,2));
  //Serial.print(dist);
  //Serial.print("\t");
  float orientation = atan2(dp3,dp1);
  triang[0] = dist;
  triang[1] = orientation*180/3.14;
}

double getOrientationInfo(int d1, int d2) {
  return atan2(d2,d1);
}

int numNans = 0;

void getFilteredDist() {
  prevDist = ftriang[0];
  if (justStartedD) {
    if (!isnan(triang[0])) {
      prevDist = triang[0];
      justStartedD = 0;
      ftriang[0] = triang[0];
      numNans = 0;
      return;
    }
    numNans++;
    ftriang[0] = 0;
    return;
  }
  if (!isnan(triang[0])) {
    ftriang[0] = BETAD * prevDist + (1 - BETAD) * triang[0];
    numNans = 0;
  } else {
    numNans++;
  }
  return;
}
  
void getFilteredOrientation() {
  prevOrientation = ftriang[1];
  if (justStartedO) {
    if (!isnan(triang[1])) {
      prevOrientation = triang[1];
      justStartedO = 0;
      ftriang[1] = triang[1];
      return;
    }
    ftriang[1] = 0;
    return;
  }
  if (!isnan(triang[1])) {
    ftriang[1] = BETAO * prevOrientation + (1 - BETAO) * triang[1];
  }
  return;
  
}

void pulseInMult(unsigned long *pulses, uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t state, unsigned long timeout)
{
	
	// cache the port and bit of the pin in order to speed up the
	// pulse width measuring loop and achieve finer resolution.  calling
	// digitalRead() instead yields much coarser resolution.

	unsigned long width = 0; // keep initialization out of time critical area
	
	// convert the timeout from microseconds to a number of times through
	// the initial loop; it takes 16 clock cycles per iteration.

        int numloops = 0;
        
        int width1 = 0;
        int width2 = 0;
        int width3 = 0;

	unsigned long maxloops = microsecondsToClockCycles(timeout) / 16;
	
	// wait for any previous pulse to end
	while (digitalReadFast(pin1) == state) {
		if (numloops++ == maxloops)
			return;
	}

        //Serial.print("stage 1: ");
        //Serial.print(numloops);

	// wait for the pulse to start
	while (digitalReadFast(pin1) != state) {
		if (numloops++ == maxloops)
			return;
	}

        //Serial.print("\tstage 2: ");
        //Serial.print(numloops);

        char endthis = 0;

        int dummy = 0;
        int dummy1 = 0;

	// wait for the pulse to stop
	while (!endthis) {
                endthis = 1;
                if (digitalReadFast(pin1) == state) {
                  endthis = 0;
                  width1++;
                }
                else {
                  dummy++;
                  dummy1++;
                }
                if (digitalReadFast(pin2) == state) {
                  endthis = 0;
                  width2++;
                }
                else {
                  dummy++;
                  dummy1++;
                }
                if (digitalReadFast(pin3) == state) {
                  endthis = 0;
                  width3++;
                }
                else {
                  dummy++;
                  dummy1++;
                }
		if (numloops++ == maxloops)
			return;
	}

	// convert the reading to microseconds. There will be some error introduced by
	// the interrupt handlers.
        //return clockCyclesToMicroseconds(width * 21 + 16);
	// Conversion constants are compiler-dependent, different compiler versions
	// have different levels of optimization.
#if __GNUC__==4 && __GNUC_MINOR__==3 && __GNUC_PATCHLEVEL__==2
	// avr-gcc 4.3.2
	pulses[0] = (width1 * 40 + 16) / 58;
        pulses[1] = (width2 * 40 + 16) / 58;
        pulses[2] = (width3 * 40 + 16) / 58;
        return;
#elif __GNUC__==4 && __GNUC_MINOR__==8 && __GNUC_PATCHLEVEL__==1
	// avr-gcc 4.8.1
	pulses[0] = (width1 * 40 + 16) / 58;
        pulses[1] = (width2 * 40 + 16) / 58;
        pulses[2] = (width3 * 40 + 16) / 58;
        return;
#elif __GNUC__<=4 && __GNUC_MINOR__<=3
	// avr-gcc <=4.3.x
	#warning "pulseIn() results may not be accurate"
	pulses[0] = (width1 * 40 + 16) / 58;
        pulses[1] = (width2 * 40 + 16) / 58;
        pulses[2] = (width3 * 40 + 16) / 58;
        return;
#else
	// avr-gcc >4.3.x
	#warning "pulseIn() results may not be accurate"
	pulses[0] = (width1 * 24 + 16) / 58;
        pulses[1] = (width2 * 24 + 16) / 58;
        pulses[2] = (width3 * 24 + 16) / 58;
        return;
#endif

}

int pinput1 = 31;
int pinput2 = 35;
int pinput3 = 39;

unsigned long distdata[3];

void pulseOut(int pin, int us)
{
   digitalWriteFast(pin, HIGH);
   us = max(us - 20, 1);
   delayMicroseconds(us);
   digitalWriteFast(pin, LOW);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  vw_set_ptt_inverted(true);
  vw_setup(2400);
  pinMode(pinput1, INPUT);
  pinMode(pinput2, INPUT);
  pinMode(pinput3, INPUT);
  digitalWrite(pinput1, LOW);
  digitalWrite(pinput2, LOW);
  digitalWrite(pinput3, LOW);
  
}

const char *msg = "1";

void loop() {
  // put your main code here, to run repeatedly:
  
  
  vw_send((uint8_t *)msg, strlen(msg));
  delayMicroseconds(20);
  pinMode(pinput1, OUTPUT);
  pinMode(pinput2, OUTPUT);
  pinMode(pinput3, OUTPUT);
  pulseOut(pinput1, 0);
  pulseOut(pinput2, 0);
  pulseOut(pinput3, 0);
  pinMode(pinput1, INPUT);
  pinMode(pinput2, INPUT);
  pinMode(pinput3, INPUT);
  
  pulseInMult(distdata, pinput1, pinput2, pinput3, HIGH, 1000000);
  
  getLocInfo(distdata[2], distdata[0], distdata[1]);
  
  getFilteredDist();
  getFilteredOrientation();
  
  float dist = ftriang[0];
  float ang = ftriang[1];
  
  for (int i = 0; i < 3; i++) {
    Serial.print(distdata[i]);
    Serial.print("\t");
  } 
 
  Serial.print(dist);
  Serial.print("\t");
  Serial.print(ang);
  
  Serial.print("\n");
}
