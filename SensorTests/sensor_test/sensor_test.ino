#include <VirtualWire.h>
#include <VirtualWire_Config.h>
#include <stdlib.h>

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

char debug = 1;

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

float prevDist;
float prevOrientation;

void drive(double spd, int turndir, double degree, int dir) {
  if (turndir == RIGHT) {
    if (dir == FORWARD) {
      analogWrite(pwm1, (unsigned char) (spd * 255));
      analogWrite(pwm2, (unsigned char) (degree * spd * 255));
    }
    else {
      analogWrite(pwm1, (unsigned char) (degree * spd * 255));
      analogWrite(pwm2, (unsigned char) (spd * 255));
    }
  }
  if (turndir == LEFT) {
    if (dir == FORWARD) {
      analogWrite(pwm1, (unsigned char) (degree * spd * 255));
      analogWrite(pwm2, (unsigned char) (spd * 255));
    }
    else {
      analogWrite(pwm1, (unsigned char) (spd * 255));
      analogWrite(pwm2, (unsigned char) (degree * spd * 255));
    }
  }
  if (dir == BACKWARD) {
    digitalWrite(dir11, HIGH);
    digitalWrite(dir12, LOW);
    digitalWrite(dir21, LOW);
    digitalWrite(dir22, HIGH);
  }
  else {
    digitalWrite(dir11, LOW);
    digitalWrite(dir12, HIGH);
    digitalWrite(dir21, HIGH);
    digitalWrite(dir22, LOW);
  }
}

void getFilteredDist() {
  if (justStartedD) {
    if (!isnan(triang[0])) {
      prevDist = triang[0];
      justStartedD = 0;
      ftriang[0] = triang[0];
      return;
    }
    ftriang[0] = 0;
    return;
  }
  if (!isnan(triang[0])) {
    prevDist = BETAD * prevDist + (1 - BETAD) * triang[0];
  }
  ftriang[0] = prevDist;
  return;
}
  
void getFilteredOrientation() {
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
    prevOrientation = BETAO * prevOrientation + (1 - BETAO) * triang[1];
  }
  ftriang[1] = prevOrientation;
  return;
  
}
  
void getToHuman(double dist, double ang) {
  
  //Distance
  double P = .02;
  double I = 0;
  double D = 0;//P * dt / 8;

  double maxError;

  if (dist > optimalDistance) {
    maxError = 100;
  } else {
    maxError = 20;
  }
  
  
  double maxControl = P * maxError;
  
  double error = dist - optimalDistance;
  Serial.print(error);
  Serial.print("\t");
  double prevError = prevDist - optimalDistance;
  totalErrorDist += error;
  
  double derivativeError = (error - prevError) / dt;
  
  double totalDistControl = P * error + I * totalErrorDist + D * derivativeError;
  
  Serial.print(fabs(totalDistControl));
  Serial.print("\t");
  
  double spd = (double) fabs(totalDistControl) / maxControl;
  
  Serial.print(spd);
  Serial.print("\n");
  
  if (isnan(spd)) {
    spd = 0;
  }
  if (spd > 0.5) {
    spd = 0.5;
  }
  
  
  
  //Orientation
  P = 0.2;
  I = 0;
  D = 0;//P * dt / 8;
  
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
    spd = 0;
  }
  else {
    forwardorback = FORWARD;
  }
  drive(spd, dir, deg, forwardorback);
}


void pulseOut(int pin, int us)
{
   digitalWrite(pin, HIGH);
   us = max(us - 20, 1);
   delayMicroseconds(us);
   digitalWrite(pin, LOW);
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
  pinMode(wireless, OUTPUT);
}

int getTime(int pin) {
  //delay(4);
  pinMode(pin, OUTPUT);
  pulseOut(pin, 0);
  pinMode(pin, INPUT);
  digitalWrite(pin, LOW);
  return pulseIn(pin, HIGH)/29;
  
}

//float *triang;

void loop() {
  //vw_setup(2400);
  const char *msg2 = "1";
  const char *msg3 = "2";
  digitalWrite(13,HIGH);
  vw_send((uint8_t *)msg2, strlen(msg2));
  vw_send((uint8_t *)msg2, strlen(msg2));
  digitalWrite(13,LOW);
  //duration = getTime(pinput1);
  //digitalWrite(wireless, LOW);
  //delay(20);
  
  //digitalWrite(wireless, HIGH);
  //delay(10);
  
  //vw_wait_tx(); // Wait until the whole message is gone
  duration = getTime(pinput1);
  //digitalWrite(wireless, LOW);
  //delay(20);
  //delay(500);
  //vw_wait_tx();
  digitalWrite(13,HIGH);
  vw_send((uint8_t *)msg2, strlen(msg2));
  //vw_send((uint8_t *)msg2, strlen(msg2));
  digitalWrite(13,LOW);
  //vw_wait_tx();
  //digitalWrite(wireless, HIGH);
  //delay(10);
  
  duration1 = getTime(pinput2);
  //digitalWrite(wireless, LOW);
  //delay(20);
  //delay(500);
  //vw_wait_tx();
  digitalWrite(13,HIGH);
  vw_send((uint8_t *)msg3, strlen(msg3));
  //vw_send((uint8_t *)msg2, strlen(msg2));
  digitalWrite(13,LOW);
  //digitalWrite(wireless, HIGH);
  //delay(10);
  
  duration2 = getTime(pinput3);
  
  //digitalWrite(wireless, LOW);
  //delay(10);
  getLocInfo(duration2, duration, duration1);
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
    Serial.print("\tTotal Distance: ");
    Serial.print(dist);
    Serial.print("\tOrientation: ");
    Serial.print(ang);
    
    Serial.print("\t");
    Serial.print(digitalRead(dir11));
    Serial.print("\t");
    Serial.print(digitalRead(dir12));
    Serial.print("\t");
    Serial.print(digitalRead(dir21));
    Serial.print("\t");
    Serial.print(digitalRead(dir22));

    
    Serial.print("\n"); 
  }
  //driver.init();
  //driver.setM1Speed(-50);
  //delay(10);
  
  getToHuman(dist, ang);
  
  String message = "";
  message += dist;
  message += '\t';
  message += ang;
  char messagechar[20];
  
  message.toCharArray(messagechar, 20);
  
  Serial.println(messagechar);
  
  vw_send((uint8_t *)messagechar, strlen(messagechar));
  
  //drive(0.3,RIGHT,1,BACKWARD);
  
}
