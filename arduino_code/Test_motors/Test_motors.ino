int pin_motor_derecho = 3;
int pin_motor_izquierdo = 6;

int s_izq_1 = 11;
int s_izq_2 = 10;

int s_der_1 = 4;
int s_der_2 = 5;

int vel = 100;

void setup() {
  // put your setup code here, to run once:
  pinMode(pin_motor_derecho, OUTPUT);
  pinMode(pin_motor_izquierdo, OUTPUT);

  pinMode(s_izq_1, OUTPUT);
  pinMode(s_izq_2, OUTPUT);

  pinMode(s_der_1, OUTPUT);
  pinMode(s_der_2, OUTPUT);

  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // Inciamos el baile

  digitalWrite(s_izq_1,HIGH);
  digitalWrite(s_izq_2,LOW);
  digitalWrite(s_der_1,HIGH);
  digitalWrite(s_der_2,LOW);
  digitalWrite(13,HIGH);
  
  analogWrite(pin_motor_izquierdo,vel);
  delay(5000);
  analogWrite(pin_motor_izquierdo,0);
  
  digitalWrite(13,LOW);
  analogWrite(pin_motor_derecho,vel);
  delay(5000);
  analogWrite(pin_motor_derecho,0);

  digitalWrite(s_izq_1,LOW);
  digitalWrite(s_izq_2,HIGH);
  digitalWrite(s_der_1,LOW);
  digitalWrite(s_der_2,HIGH);

  digitalWrite(13,HIGH);
  analogWrite(pin_motor_izquierdo,vel);
  delay(5000);
  analogWrite(pin_motor_izquierdo,0);
  
  digitalWrite(13,LOW);
  analogWrite(pin_motor_derecho,vel);
  delay(5000);
  analogWrite(pin_motor_derecho,0);

  

  
  
  
 

}
