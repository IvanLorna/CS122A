IntervalTimer myTimer;

void setup() {
  //pinMode(A0,OUTPUT);
  //digitalWrite(A0,HIGH); 
  serial.begin(9600);  //FIXME::add serial library
  //attachInterrupt(digitalPinToInterrupt(A1), Work, RISING);
  myTimer.begin(Work, 1500000);
}

void Work() {
  if (A0 == HIGH) {digitalWrite(A0,LOW);}
  else { digitalWrite(A0, HIGH);}  
  serial.print( " :^) ");
}


void loop() {
  idle; //FIXME:: add idle/sleep library
}
