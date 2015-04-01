#include <VirtualWire.h>
#include <VirtualWire_Config.h>
#include <stdlib.h>
#include <math.h>

#define XDIST (12.25*25.54)
#define YDIST (6.65*25.54)
#define ZpDIST (12.35*25.54)
#define ZDIST (sqrt(pow(YDIST,2)+pow(ZpDIST,2)))
#define BETAD 0.6
#define BETAO 0.6
#define LEFT 1
#define RIGHT 2
#define FORWARD 3
#define BACKWARD 4



//CHANGE IF YOU WANT TO DEBUG
char debug = 0;


int pinput1 = 2;
int pinput2 = 3;
int poutput = 13;
int leftmotorspeed = 5;
int rightmotorspeed = 6;
int leftmotordir = 7;
int rightmotordir = 4;
unsigned long duration;
unsigned long duration1;

char justStartedD = 1;
char justStartedO = 1;

float prevDist;
float prevOrientation;

double totalErrorDist = 0;
double totalErrorAngle = 0;
double optimalDistance = 60;
double dt = 0.02;

double prevTime = 0.0;

void drive(double spd, int turndir, double degree, int dir) {
  if (turndir == RIGHT) {
    if (dir == FORWARD) {
      analogWrite(leftmotorspeed, (unsigned char) (spd * 255));
      analogWrite(rightmotorspeed, (unsigned char) (degree * spd * 255));
    }
    else {
      analogWrite(leftmotorspeed, (unsigned char) (degree * spd * 255));
      analogWrite(rightmotorspeed, (unsigned char) (spd * 255));
    }
  }
  if (turndir == LEFT) {
    if (dir == FORWARD) {
      analogWrite(leftmotorspeed, (unsigned char) (degree * spd * 255));
      analogWrite(rightmotorspeed, (unsigned char) (spd * 255));
    }
    else {
      analogWrite(leftmotorspeed, (unsigned char) (spd * 255));
      analogWrite(rightmotorspeed, (unsigned char) (degree * spd * 255));
    }
  }
  if (dir == FORWARD) {
    digitalWrite(leftmotordir, HIGH);
    digitalWrite(rightmotordir, HIGH);
  }
  else {
    digitalWrite(leftmotordir, LOW);
    digitalWrite(rightmotordir, LOW);
  }
}

void getToHuman(double dist, double ang) {
  
  //Distance
  double P = 1;
  double I = 0;
  double D = 0;

  double maxError;

  if (dist > optimalDistance) {
    maxError = 30;
  } else {
    maxError = 20;
  }
  
  double maxControl = P * maxError;
  
  double error = dist - optimalDistance;
  double prevError = prevDist - optimalDistance;
  totalErrorDist += error;
  
  double derivativeError = (error - prevError) / dt;
  
  double totalDistControl = P * error + I * totalErrorDist + D * derivativeError;
  
  double spd = abs(totalDistControl) / maxControl;
  if (isnan(spd)) {
    spd = 0;
  }
  if (spd > 1.0) {
    spd = 1.0;
  }
  
  
  
  //Orientation
  P = 1;
  I = 0;
  D = 0;
  
  maxError = 45;
  
  maxControl = P * maxError;
  
  error = abs(ang);
  prevError = abs(prevDist);
  totalErrorAngle += error;
  
  derivativeError = (error - prevError) / dt;
  
  double totalAngleControl = P * error + I * totalErrorAngle + D * derivativeError;
  
  double deg = 1 - (totalAngleControl / maxControl);
  
  if (isnan(spd)) {
    spd = 0;
  }
  if (spd > 1.0) {
    spd = 1.0;
  }
  if (spd < 0.0) {
    spd = 0.0;
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
  
  char forwardorback;
  
  if (dist < optimalDistance) {
    forwardorback = BACKWARD;
  }
  else {
    forwardorback = FORWARD;
  }
  drive(spd, dir, deg, forwardorback);
}

double getLocInfo(int d1, int d2) {
  double cosbeta = (pow(d2,2)+pow(DIST_BETWEEN_SENSORS,2)-pow(d1,2))/(2*d2*DIST_BETWEEN_SENSORS);
  double beta = atan2(sqrt(1-pow(cosbeta,2)), cosbeta);
  double d = sqrt(pow(sin(beta),2)*pow(d2,2)+pow(DIST_BETWEEN_SENSORS/2-cos(beta)*d2,2));
  return d;
}

double getOrientationInfo(double d, int d1, int d2) {
  double cosbeta = (pow(d2,2)+pow(DIST_BETWEEN_SENSORS,2)-pow(d1,2))/(2*d2*DIST_BETWEEN_SENSORS);
  double beta = atan2(sqrt(1-pow(cosbeta,2)), cosbeta);
  double sintheta = sin(beta)*d2/d;
  double theta = 3.141592/2 - atan2(sintheta, sqrt(1-pow(sintheta,2)));
  if (d2 < d1) {
    theta = -1 * theta;
  }
  return theta * 180 / 3.14;
}

double getFilteredDist(float d) {
  if (justStartedD) {
    if (!isnan(d)) {
      prevDist = d;
      justStartedD = 0;
      return d;
    }
    return 0;
  }
  if (!isnan(d)) {
    prevDist = BETAD * prevDist + (1 - BETAD) * d;
  }
  return prevDist;
}
  
double getFilteredAngle(float a) {
  if (justStartedO) {
    if (!isnan(a)) {
      prevOrientation = a;
      justStartedO = 0;
      return a;
    }
    return 0;
  }
  if (!isnan(a)) {
    prevOrientation = BETAO * prevOrientation + (1 - BETAO) * a;
  }
  return prevOrientation;
}

void pulseOut(int pin, int us)
{
   digitalWrite(pin, HIGH);
   us = max(us - 20, 1);
   delayMicroseconds(us);
   digitalWrite(pin, LOW);
}

void setup() {
  if (debug) {
    Serial.begin(4800);	  // Debugging only
  }

  // Initialise the IO and ISR
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_setup(2400);	 // Bits per sec
  pinMode(pinput1, INPUT);
  pinMode(pinput2, INPUT);
  pinMode(leftmotorspeed, OUTPUT);
  pinMode(rightmotorspeed, OUTPUT);
  pinMode(leftmotordir, OUTPUT);
  pinMode(rightmotordir, OUTPUT);
  digitalWrite(pinput1, LOW);
  digitalWrite(pinput2, LOW);
  analogWrite(leftmotorspeed, 255);
  analogWrite(rightmotorspeed, 255);
  digitalWrite(leftmotordir, HIGH);
  digitalWrite(rightmotordir, HIGH);
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
  double filteredDist = getFilteredDist(dist);
  double filteredAng = getFilteredAngle(ang);
  
  if (debug) {
    Serial.print(duration);
    Serial.print("\t");
    Serial.print(duration1);
    Serial.print("\tTotal Distance: ");
    Serial.print(dist);
    Serial.print("\tFiltered Distance: ");
    Serial.print(filteredDist);
    Serial.print("\tOrientation: ");
    Serial.print(ang);
    Serial.print("\tFiltered Orientation: ");
    Serial.print(filteredAng);
  }
  
  getToHuman(filteredDist, filteredAng);
  
  if (debug) {
    Serial.print("\n");
  }
  
  while(micros() < prevTime + dt * 1000000);
  prevTime = micros();
  
}
