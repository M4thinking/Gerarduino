// Pin de conxión a los motores
int pin_motor_izq = 5;
int pin_motor_der = 6;

// Pin de lectura sensores
int pin_sensor_izq = A2;
int pin_sensor_der = A0;

int lectura_izquierda;
int lectura_derecha;

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
  referencia = lectura_derecha-lectura_izquierda;
  Serial << "Motor izq: " << lectura_izquierda << '\t';
  Serial << "Motor der: " << lectura_derecha << '\t';
  Serial << "Ref: " << referencia;
  Serial.println();
  if (0 < referencia ){
    analogWrite(pin_motor_izq,200);
    analogWrite(pin_motor_der,200 + abs(referencia));
  }
  else{
    analogWrite(pin_motor_der,200);
    analogWrite(pin_motor_izq, 200 + abs(referencia));
 }
  delay(10);
}
