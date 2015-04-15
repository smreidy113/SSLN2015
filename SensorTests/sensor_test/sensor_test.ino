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

int timerpin = 45;

char debug = 0;

//#include <VirtualWire.h>
#include <stdlib.h>

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

double getOrientationInfo(int d1, int d2) {
  return atan2(d2,d1);
}

int pinput1 = 31;
int pinput2 = 35;
int pinput3 = 39;
int wireless = 12;
unsigned long duration;
unsigned long duration1;
unsigned long duration2;

char justStartedD = 1;
char justStartedO = 1;

double totalErrorDist = 0;
double totalErrorAngle = 0;
double optimalDistance = 100;
double dt = 1.0/12;

int pwm1 = 7;
int pwm2 = 6;
int dir11 = 22;
int dir12 = 23;
int dir21 = 24;
int dir22 = 25;
int en1 = 26;
int en2 = 27;

float prevDist;
float prevOrientation;

float prevSpd = 0;

int leftspeed = 0;
int rightspeed = 0;

unsigned long distdata[3];

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

void spike(int dir) {
  drive(150.0/255, RIGHT, 1, dir);
  delay(10);
}

void drive(double spd, int turndir, double degree, int dir) {
  if (turndir == RIGHT) {
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
  if (turndir == LEFT) {
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
    analogWrite(pwm1, leftspeed + 8);
  } else {
    analogWrite(pwm1, leftspeed + 1);
  }
  analogWrite(pwm2, rightspeed);
  
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
  
void getToHuman(double dist, double ang) {
  
  //Distance
  double P = 0.004;
  double I = 0;
  double D = 0.01;

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
  Serial.print(spd);
  Serial.print("\n");
  }
  
  if (isnan(spd)) {
    spd = 0; 
  }
  if (spd > 0.4) {
    spd = 0.4;
  }
  if (spd < -0.4) {
    spd = -0.4;
  }
  
  if (numNans > 10 || dist <= 0.00) {
    spd = 0;
  }
  
  prevSpd = spd;
  
  //Orientation
  P = 0;
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


void pulseOut(int pin, int us)
{
   digitalWriteFast(pin, HIGH);
   us = max(us - 20, 1);
   delayMicroseconds(us);
   digitalWriteFast(pin, LOW);
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
  pinMode(wireless, OUTPUT);
  digitalWrite(en1, HIGH);
  digitalWrite(en2, HIGH);
  
  pinMode(timerpin, OUTPUT);
  
}

int getTime(int pin) {
  //delay(4);
  pinMode(pin, OUTPUT);
  pulseOut(pin, 0);
  pinMode(pin, INPUT);
  digitalWriteFast(pin, LOW);
  return pulseIn(pin, HIGH)/29;
  
}

//float *triang;
const char *msg = "1";

void loop() {
  
  if (digitalReadFast(timerpin)) {
    digitalWriteFast(timerpin, LOW);
  }
  else {
    digitalWriteFast(timerpin, HIGH);
  }
  
  //vw_setup(2400);
  const char *msg2 = "1";
  const char *msg3 = "2";
//  digitalWrite(13,HIGH);
//  vw_send((uint8_t *)msg2, strlen(msg2));
//  vw_send((uint8_t *)msg2, strlen(msg2));
//  digitalWrite(13,LOW);
//  //duration = getTime(pinput1);
//  //digitalWrite(wireless, LOW);
//  //delay(20);
//  
//  //digitalWrite(wireless, HIGH);
//  //delay(10);
//  
//  //vw_wait_tx(); // Wait until the whole message is gone
//  duration = getTime(pinput1);
//  //digitalWrite(wireless, LOW);
//  //delay(20);
//  //delay(500);
//  //vw_wait_tx();
//  digitalWrite(13,HIGH);
//  vw_send((uint8_t *)msg2, strlen(msg2));
//  //vw_send((uint8_t *)msg2, strlen(msg2));
//  digitalWrite(13,LOW);
//  //vw_wait_tx();
//  //digitalWrite(wireless, HIGH);
//  //delay(10);
//  
//  duration1 = getTime(pinput2);
//  //digitalWrite(wireless, LOW);
//  //delay(20);
//  //delay(500);
//  //vw_wait_tx();
//  digitalWrite(13,HIGH);
//  vw_send((uint8_t *)msg3, strlen(msg3));
//  //vw_send((uint8_t *)msg2, strlen(msg2));
//  digitalWrite(13,LOW);
//  //digitalWrite(wireless, HIGH);
//  //delay(10);
//  
//  duration2 = getTime(pinput3);
//  
//  //digitalWrite(wireless, LOW);
  //delay(10);
  
  
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
  //getLocInfo(132,84,134);
  float dist = ftriang[0];
  float ang = ftriang[1];
  if (debug) {
    Serial.print(duration);
    Serial.print("\t");
    Serial.print(duration1);
    Serial.print("\t");
    Serial.print(duration2);
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
  //driver.init();
  //driver.setM1Speed(-50);
  //delay(10);
  
  getToHuman(dist, ang);
  
  if (debug) {
  String message = "";
  message += dist;
  message += '\t';
  message += ang;
  message += '\t';
  message += leftspeed;
  message += '\t';
  message += rightspeed;
  message += '\t';
  message += prevDist;
  char messagechar[40];
  
  message.toCharArray(messagechar, 40);
  
  Serial.println(messagechar);
  
  vw_send((uint8_t *)messagechar, strlen(messagechar));
  }
  //drive(0.3,RIGHT,1,BACKWARD);
  
}
