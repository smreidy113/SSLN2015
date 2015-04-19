#include <VirtualWire.h>
#include <VirtualWire_Config.h>
//#include <funcs.h>
#include <stdlib.h>
#include <digitalWriteFast.h>

#define DIST_BETWEEN_SENSORS 22.3
#define BETAD 0.1
#define BETAO 0.1
#define LEFT 1
#define RIGHT 2
#define FORWARD 3
#define BACKWARD 4
#define X 26.0
#define Y 32.0
#define ZP 52.5

char debug = 0;
char pton = 0;
int timerpin = 45;
float topspeed = 0.6;

int pinput1 = 31;
int pinput2 = 35;
int pinput3 = 39;

int pwm1 = 7;
int pwm2 = 6;
int dir11 = 23;
int dir12 = 22;
int dir21 = 25;
int dir22 = 24;
int en1 = 26;
int en2 = 27;

char wirelesscomm = 1;

double optimalDistance = 100;
double dt = 1.0/40;

/***

PT SENSING: We analogread all this stuff in, and depending on the values we know
that we're in either a good or bad place.

***/

int analogPins[14] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13};

unsigned short FL[3];
unsigned short FR[3];
unsigned short RL[3];
unsigned short RR[3];
unsigned short BL;
unsigned short BR;

void readPT() {
  FL[0] = analogRead(analogPins[0]);
  FL[1] = analogRead(analogPins[1]);
  FL[2] = analogRead(analogPins[2]);
  
  FR[0] = analogRead(analogPins[3]);
  FR[1] = analogRead(analogPins[4]);
  FR[2] = analogRead(analogPins[5]);
  
  RL[0] = analogRead(analogPins[6]);
  RL[1] = analogRead(analogPins[7]);
  RL[2] = analogRead(analogPins[8]);
  
  RR[0] = analogRead(analogPins[9]);
  RR[1] = analogRead(analogPins[10]);
  RR[2] = analogRead(analogPins[11]);
  
  BL = analogRead(analogPins[12]);
  BR = analogRead(analogPins[13]);
}

char obstacle() {
  readPT();
  
  int thresh = 100;
  
  if (FL[0] > thresh || FL[1] > thresh || FL[2] > thresh) {
    return 1;
  }
  
  if (FR[0] > thresh || FR[1] > thresh || FR[2] > thresh) {
    return 2;
  }
  
  if (RL[0] > thresh || RL[1] > thresh || RL[2] > thresh) {
    return 3;
  }
  
  if (RR[0] > thresh || RR[1] > thresh || RR[2] > thresh) {
    return 4;
  } 
  
  if (BL > thresh) {
    return 5;
  }
  
  if (BR > thresh) {
    return 6;
  }
  
  return 0;
  
}

/** STAIRS DETECTION

PATRIK, THE CODE IS HERE

**/

unsigned int normalFloor;

void calibrate() {
 
 unsigned int sumPT = 0;
  
 for (int i = 0; i < 20; i++) {
   readPT();
   sumPT += BR;
 }
 
 normalFloor = sumPT / 20;
  
}

const int PTCapacity = 10;

unsigned int prevStairVals[PTCapacity];
unsigned int stairValsTotal;

int numPTSamples;

int stairsThresh;

char stairs() {
  
  if (numPTSamples < PTCapacity) {
    prevStairVals[numPTSamples] = BR;
    numPTSamples++;
    
    if (numPTSamples == PTCapacity) {
      for (int i = 0; i < PTCapacity; i++) {
        stairValsTotal += prevStairVals[i];
      }
    }
    return 0;
  }
  
  stairValsTotal -= prevStairVals[0];
  
  for (int i = 0; i < PTCapacity - 1; i++) {
    prevStairVals[i] = prevStairVals[i-1];
  }
  
  prevStairVals[PTCapacity - 1] = BR;
  
  stairValsTotal += BR;
  
  if (abs((stairValsTotal / PTCapacity) - normalFloor) > stairsThresh) {
    return 1;
  }
  
  return 0;
  
}

/***

TRIANGULATION: All the code for determining distance/orientation info, filtering it,
etc. goes here.

***/

float Z = sqrt(pow(Y,2)+pow(ZP,2));
float t = atan2(ZP,Y);
float cost = cos(t);
float sint = sin(t);

float triang[2];
float ftriang[2];

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

//Filtering the data happens here.

double getOrientationInfo(int d1, int d2) {
  return atan2(d2,d1);
}

char justStartedD = 1;
char justStartedO = 1;

float prevDist;
float prevOrientation;

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

/***

ULTRASONIC SENSING: Handling all the wait times and syncing the ping detecting goes here.

***/

unsigned long distdata[3];

void pulseOut(int pin, int us)
{
   digitalWriteFast(pin, HIGH);
   us = max(us - 20, 1);
   delayMicroseconds(us);
   digitalWriteFast(pin, LOW);
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

/***

DRIVING CONTROL: Once we know the speed and turn radius we want, we handle delivering
the voltage to the motor drivers here.

***/

int leftspeed = 0;
int rightspeed = 0;

void brake() {
  analogWrite(pwm1, 128);
  analogWrite(pwm2, 128);
  digitalWriteFast(dir11, HIGH);
  digitalWriteFast(dir12, HIGH);
  digitalWriteFast(dir21, HIGH);
  digitalWriteFast(dir22, HIGH);
}

void spike(int dir) {
  drive(150.0/255, RIGHT, 1, dir);
  delay(10);
}

void drive(double spd, int turndir, double degree, int dir) {
  if (turndir == LEFT) {
    if (dir == FORWARD) {
      leftspeed = (unsigned char) (spd * 255);
      rightspeed = (unsigned char) (degree * spd * 255);
      //analogWrite(pwm1, (unsigned char) (spd * 255));
      //analogWrite(pwm2, (unsigned char) (degree * spd * 255));
    }
    else {
      leftspeed = (unsigned char) (degree * spd * 255);
      rightspeed = (unsigned char) (spd * 255);
      //analogWrite(pwm1, (unsigned char) (degree * spd * 255));
      //analogWrite(pwm2, (unsigned char) (spd * 255));
    }
  }
  if (turndir == RIGHT) {
    if (dir == FORWARD) {
      leftspeed = (unsigned char) (degree * spd * 255);
      rightspeed = (unsigned char) (spd * 255);
      //analogWrite(pwm1, (unsigned char) (degree * spd * 255));
      //analogWrite(pwm2, (unsigned char) (spd * 255));
    }
    else {
      leftspeed = (unsigned char) (spd * 255);
      rightspeed = (unsigned char) (degree * spd * 255);
      //analogWrite(pwm1, (unsigned char) (spd * 255));
      //analogWrite(pwm2, (unsigned char) (degree * spd * 255));
    }
  }
  if (dir == FORWARD) {
    analogWrite(pwm2, rightspeed + 4);
  } else {
    analogWrite(pwm2, rightspeed + 1);
  }
  analogWrite(pwm1, leftspeed);
  
  if (dir == BACKWARD) {
    digitalWriteFast(dir11, LOW);
    digitalWriteFast(dir12, HIGH);
    digitalWriteFast(dir21, HIGH);
    digitalWriteFast(dir22, LOW);
    
    leftspeed = -1 * leftspeed;
    rightspeed = -1 * rightspeed;
  }
  else {
    digitalWriteFast(dir11, HIGH);
    digitalWriteFast(dir12, LOW);
    digitalWriteFast(dir21, LOW);
    digitalWriteFast(dir22, HIGH);
  }
}

/***

CONTROLS: Handling dist/orientation error compensation happens here.

***/

double totalErrorDist = 0;
double totalErrorAngle = 0;

float prevSpd = 0;
  
void getToHuman(double dist, double ang) {
  
  //Distance
  double P = 0.007;
  double I = 0;//0.00002;
  double D = 0.003;

  double maxError;

  if (dist > optimalDistance) {
    maxError = 100;
  } else {
    maxError = 20;
  }
  
  
  double maxControl = P * maxError;
  
  double error = dist - optimalDistance;
  
  if (debug) {
  Serial.print(error);
  Serial.print("\t");
  }
  
  double prevError = prevDist - optimalDistance;
  totalErrorDist += error;
  
  double derivativeError = (error - prevError) / dt;
  
  double totalDistControl = P * error + I * totalErrorDist + D * derivativeError;
  
  if (debug){
  Serial.print(fabs(totalDistControl));
  Serial.print("\t");
  }
  
  double spdchange = (double) (totalDistControl);
  
  double spd = prevSpd + spdchange;
  
  int thresh = 0.2;
  
  if ((prevSpd < thresh && spd > thresh) || (prevSpd > -1*thresh && spd < -1*thresh)) {
    if (spd > 0) {
      spike(FORWARD);
    } else {
      spike(BACKWARD);
    }
  }
  
  if (debug) {
  Serial.print("speed: ");
  Serial.print(spd);
  //Serial.print("\n");
  }
  
  if (isnan(spd)) {
    spd = 0; 
  }
  if (spd > topspeed) {
    spd = topspeed;
  }
  if (spd < -1*topspeed) {
    spd = -1*topspeed;
  }
  
  if (numNans > 50 || dist <= 0.00) {
    spd = 0;
  }
  
  prevSpd = spd;
  
  //Orientation
  P = 0.03;
  I = 0;
  D = 0;//P * dt / 8;
  
  maxError = 45;
  
  maxControl = P * maxError;
  
  error = abs(ang);
  prevError = abs(prevOrientation);
  totalErrorAngle += error;
  
  derivativeError = (error - prevError) / dt;
  
  double totalAngleControl = P * error + I * totalErrorAngle + D * derivativeError;
  
  double deg = 1 - (totalAngleControl);
  if (deg < 0) deg = 0;
  
  char forwardorback;
  
  if (isnan(spd)) {
    spd = 0;
  }
  if (spd > 1.0) {
    spd = 1.0;
  }
  if (spd < 0.0) {
    spd = -1*spd;
    forwardorback = BACKWARD;
  } else {
    forwardorback = FORWARD;
  }
  
  int dir;
  
  if (ang < 0.0) {
    dir = LEFT;
  } else {
    dir = RIGHT;
  }
  
  if (debug) {
    Serial.print("\t");
    Serial.print(spd);
  }
  
  drive(spd, dir, deg, forwardorback);
}


void setup() {
  Serial.begin(9600);	  // Debugging only
  
  // Initialise the IO and ISR
  //vw_set_tx_pin(13);
  //vw_set_ptt_pin(1);
  //vw_set_rx_pin(0);
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_setup(2400);	 // Bits per sec
  pinMode(pinput1, INPUT);
  pinMode(pinput2, INPUT);
  pinMode(pinput3, INPUT);
  digitalWrite(pinput1, LOW);
  digitalWrite(pinput2, LOW);
  digitalWrite(pinput3, LOW);
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(dir11, OUTPUT);
  pinMode(dir12, OUTPUT);
  pinMode(dir21, OUTPUT);
  pinMode(dir22, OUTPUT);
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);
  digitalWrite(en1, HIGH);
  digitalWrite(en2, HIGH);
  
  pinMode(timerpin, OUTPUT);
  
  //calibrate();
  
}

//float *triang;
const char *msg = "1";

void loop() {
  
  
  vw_send((uint8_t *)msg, strlen(msg));
  //vw_send((uint8_t *)msg, strlen(msg));
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
  //getLocInfo(132,84,134);
  float dist = ftriang[0];
  float ang = ftriang[1];
  if (debug) {
    Serial.print(distdata[2]);
    Serial.print("\t");
    Serial.print(distdata[0]);
    Serial.print("\t");
    Serial.print(distdata[1]);
    Serial.print("\t");
    Serial.print("Prev Distance: ");
    Serial.print(prevDist);
    Serial.print("\tTotal Distance: ");
    Serial.print(dist);
    Serial.print("\tOrientation: ");
    Serial.print(ang);
    
    Serial.print("\t");
    Serial.print(digitalReadFast(dir11));
    Serial.print("\t");
    Serial.print(digitalReadFast(dir12));
    Serial.print("\t");
    Serial.print(digitalReadFast(dir21));
    Serial.print("\t");
    Serial.print(digitalReadFast(dir22));

    
    Serial.print("\n"); 
  }
  
  if (pton) {
    if (!stairs()) {
      getToHuman(dist, ang);
    } else {
      brake();
    }
  } else {
    getToHuman(dist, ang);
  }
  
  if (wirelesscomm) {
  String message = "";
  message += distdata[2];
  message += '\t';
  message += distdata[0];
  message += '\t';
  message += distdata[1];
  message += '\t';
  message += dist;
  message += '\t';
  message += ang;
  message += '\t';
  message += leftspeed;
  message += '\t';
  message += rightspeed;
  char messagechar[80];
  
  message.toCharArray(messagechar, 40);
  
  Serial.println(messagechar);
  
  //vw_send((uint8_t *)messagechar, strlen(messagechar));
  }
  //drive(0.3,RIGHT,1,BACKWARD);
  
}
