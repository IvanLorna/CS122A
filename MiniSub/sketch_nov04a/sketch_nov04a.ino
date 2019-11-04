IntervalTimer myTimer;
int counter = 0;
int LED = LOW;
int PWM = 50;
void setup() {
  pinMode(A0,OUTPUT);
  counter = 0;
  PWM = 25;
  LED = LOW; 
  Serial.begin(9600);  //FIXME::add serial library
  //attachInterrupt(digitalPinToInterrupt(A1), Work, RISING);
  myTimer.begin(Work, 15000);
}

void Work() {
  counter++;
  if (counter >= 100) {
    counter = 0;
  }
  else if (counter > PWM) {
    LED = HIGH;
  }
  else if (counter < PWM) {
    LED = LOW;
  }
  digitalWrite(A0,LED);  
  Serial.print( " :^) ");
}


void loop() {
//  idle; //FIXME:: add idle/sleep library
}
