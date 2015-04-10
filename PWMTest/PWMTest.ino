int pwm1 = 6;
int pwm2 = 7;

float val1 = 0;
float val2 = 0;

float count = 0.01;

float pwmRead(int pin) {
  int on = pulseIn(pin, HIGH);
  int off = pulseIn(pin, LOW);
  return (float) on / (on + off) * 5;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(pwm1,OUTPUT);
  pinMode(pwm2,OUTPUT);
  //pinMode(A0,INPUT);
  //pinMode(A1,INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(pwm1, count * 255);
  analogWrite(pwm2, count * 255);
  
  Serial.print((count * 255));
  Serial.print('\t');
  val1 = pwmRead(A0);
  Serial.print(val1);
  Serial.print('\t');
  val2 = pwmRead(A1);
  Serial.println(val2);
  
  if (count >= 1) {
    while(1) {}
  }
    
    delay(100);
    
    count += 0.01;
    
}
