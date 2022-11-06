int pin_derecho = 6;

void setup() {
  // put your setup code here, to run once:
  pinMode(pin_derecho,OUTPUT);
  pinMode(13,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  //analogWrite(pin_derecho,250);
  //delay(1000);
  //analogWrite(pin_derecho,100);
  //delay(1000);
  //analogWrite(pin_derecho,0);
  //delay(1000);
  digitalWrite(pin_derecho,HIGH);
  digitalWrite(13,HIGH);
  delay(3000);
  digitalWrite(pin_derecho, LOW);
  digitalWrite(13,LOW);
  delay(3000);
  
}
