
void setup() {
  pinMode(A0,OUTPUT);
  digitalWrite(A0,HIGH); 
}

void loop() {
  delay(25);
  if (A0 == HIGH) {digitalWrite(A0,LOW);}
  else { digitalWrite(A0, HIGH);}
  

}
