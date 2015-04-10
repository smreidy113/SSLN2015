#include <math.h>

int pwm1 = 6;
int pwm2 = 7;
int dir11 = 22;
int dir12 = 23;
int dir21 = 24;
int dir22 = 25;
int en1 = 26;
int en2 = 27;
//int cs1 = A8;
//int cs2 = A9;

int m1v1 = A8;
int m1v2 = A9;
int m2v1 = A10;
int m2v2 = A11;

void setup() {
  // put your setup code here, to run once:
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(dir11, OUTPUT);
  pinMode(dir12, OUTPUT);
  pinMode(dir21, OUTPUT);
  pinMode(dir22, OUTPUT);
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);
  
  digitalWrite(dir11, LOW);
  digitalWrite(dir12, HIGH);
  
  digitalWrite(dir21, HIGH);
  digitalWrite(dir22, LOW);  
  
  digitalWrite(en1, HIGH);
  digitalWrite(en2, HIGH);
  
  Serial.begin(9600);
  
}

float count = 0;
float incr = .1;

float v1 = 0;
float v2 = 0;

int m1v1o = 0;
int m1v2o = 0;
int m2v1o = 0;
int m2v2o = 0;

float c1 = 0;
float c2 = 0;

void loop() {
  // put your main code here, to run repeatedly:

  if (count >= 128) {
    incr = -.1;
  }
  
  if (count <= 0) {
    incr = .1;
  }

  analogWrite(pwm1, (unsigned char) count);
  analogWrite(pwm2, (unsigned char) count);
  
  //v1 = analogRead(cs1);
  //v2 = analogRead(cs2);
  
  //c1 = (float) v1 * 5 / 1024 / .140 * 1000;
  //c2 = (float) v2 * 5 / 1024 / .140 * 1000;
  
  m1v1o = analogRead(m1v1);
  m1v2o = analogRead(m1v2);
  m2v1o = analogRead(m2v1);
  m2v2o = analogRead(m2v2);
  
  v1 = fabs(m1v1o - m1v2o) * 15 / 1023;
  v2 = fabs(m2v1o - m2v2o) * 15 / 1023;
  
  // Serial.print("Speed: ");
  Serial.print(((float) count/255));
  Serial.print("\t");
  // Serial.print("\tM1 Current: ");
  Serial.print(v1);
  // Serial.print("\tM2 Current: ");
  Serial.print("\t");
  Serial.println(v2);
  //delay(10);
  
  count += incr;
}
