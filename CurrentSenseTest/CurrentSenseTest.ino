int pwm1 = 6;
int pwm2 = 7;
int dir11 = 22;
int dir12 = 23;
int dir21 = 24;
int dir22 = 25;
int en1 = 26;
int en2 = 27;
int cs1 = A8;
int cs2 = A9;

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
  
  digitalWrite(dir11, HIGH);
  digitalWrite(dir12, LOW);
  
  digitalWrite(dir21, HIGH);
  digitalWrite(dir22, LOW);
  
  digitalWrite(en1, HIGH);
  digitalWrite(en2, HIGH);
  
  Serial.begin(9600);
  
}

int count = 0;

int v1 = 0;
int v2 = 0;

float c1 = 0;
float c2 = 0;

void loop() {
  // put your main code here, to run repeatedly:

  if (count == 128) {
    count = 0;
  }

  analogWrite(pwm1, count);
  analogWrite(pwm2, count);
  
  v1 = analogRead(cs1);
  v2 = analogRead(cs2);
  
  c1 = (float) v1 * 5 / 1024 / 140;
  c2 = (float) v2 * 5 / 1024 / 140;
  
  Serial.print("Speed: ");
  Serial.print(((float) count/255));
  Serial.print("\tM1 Current: ");
  Serial.print(c1);
  Serial.print("\tM2 Current: ");
  Serial.println(c2);

}
