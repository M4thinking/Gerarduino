#include 'pins.h'
// Pin de conxión a los motores
int pin_motor_izq = 11;
int pin_motor_der = 10;

// Pin de lectura sensores
int pin_sensor_izq = A2;
int pin_sensor_der = A0;

int lectura_izquierda;
int lectura_derecha;
int base = 100;
int referencia;

template <typename T>
Print& operator<<(Print& printer, T value){
  printer.print(value);
  return printer;
}

void setup() {
  // Setup Pines
  pinMode(pin_motor_izq,OUTPUT);
  pinMode(pin_motor_der,OUTPUT);
  
  pinMode(pin_sensor_izq,INPUT);
  pinMode(pin_sensor_der,INPUT);
  Serial.begin(9600);
}
// Blanco ~50; Negro ~900

void loop() {
  // Leemos los sensores
  lectura_izquierda = analogRead(pin_sensor_izq);
  lectura_derecha = analogRead(pin_sensor_der);
  // Generamos la referencia
  referencia = (lectura_derecha-lectura_izquierda)/20;
  Serial << "Sensor izq: " << lectura_izquierda << '\t';
  Serial << "Sensor der: " << lectura_derecha << '\t';
  Serial << "Ref: " << referencia << '\t' << '\t';
  //Serial.println();
  if ( referencia < 0 ){
    analogWrite(pin_motor_izq,base);
    analogWrite(pin_motor_der,base + abs(referencia));
    Serial << "Motor izq: " << base << '\t';
    Serial << "Motor der: " << base + abs(referencia) << '\t';
  }
  else{
    analogWrite(pin_motor_der,base);
    analogWrite(pin_motor_izq, base + abs(referencia));
    Serial << "Motor izq: " << base +abs(referencia) << '\t';
    Serial << "Motor der: " << base  << '\t';
 }
 Serial.println();
 delay(1);
}
