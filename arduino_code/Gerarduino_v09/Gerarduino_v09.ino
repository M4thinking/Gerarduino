#include <QTRSensors.h>
QTRSensors qtr;

const uint8_t SensorCount = 6, detectionCount = 5;
uint16_t sensorValues[SensorCount], detectionValues[detectionCount] = {};

//==============================================================
// Inicializamos los pines
int pin_motor_izq = 6, pin_motor_der = 3;
int sensor_izq = 7, sensor_der = 9, LED = 13;
int enable, iteration = 0, izq, der;

const int IN1 = 4;
const int IN2 = 5;
const int IN3 = 11;
const int IN4 = 10;

//==============================================================
// Constantes del PD
float KD = 15.0;  // Constante derivativa PID
float KP = 6.5;  // Constante proporcional PID

//=============================================================
//Inicialización de las variables
int lastError = 0, error = 0, state = 0, init_time=0, detection = 0;    
int m_izq_speed, m_der_speed, pos, motorSpeed, currState;
int base_speed = 140, max_speed = 160, calibration_speed = 130;

//=============================================================
// Funciones para printear informacion rapidamente
template <typename T>
Print& operator<<(Print& printer, T value){
  printer.print(value);
  return printer;
}

//=============================================================
// Función que mueve al robot
void move_robot() {
  // Leer posición
  pos = qtr.readLineWhite(sensorValues);

  // Mapear la posición a un valor entre -100 y 100
  error = map(pos, 0, 6000, -50, 50);
  // Limitar la velocidad de los motores

  // Acción de control
  motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;

  // Limit the motor speeds to the range 0 to 255
  m_izq_speed = constrain(base_speed + motorSpeed, 0, max_speed);
  m_der_speed = constrain(base_speed - motorSpeed, 0, max_speed);

  // Hechamos a andar
  analogWrite(pin_motor_izq, m_izq_speed);
  analogWrite(pin_motor_der, m_der_speed);

//=============================================================
  //for (uint8_t i = 0; i < SensorCount; i++){Serial.print(sensorValues[i]);\
  
  
  
  
  
//=============================================================
}

//=============================================================
//=============================================================

void setup() {
  //Serial.begin(9600);
  // Configuramos los sensores
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {
   A1, A2, A3, A4, A5, A6
  }, SensorCount);
  qtr.setEmitterPin(12);

  // Pines de sensores infrarojo laterales en D7 y D9 
  pinMode(sensor_izq, INPUT);
  pinMode(sensor_der, INPUT);

  // Pines de sentido de referencia motor derecho
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Pines de sentido de referencia motor izquierdo
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  //Pin led
  pinMode(LED, OUTPUT);
  
  analogWrite(pin_motor_izq, 0);
  analogWrite(pin_motor_der, 0);

  delay(2000);

  
  // CALIBRAMOS
  //Serial << "Calibrando..."<<'\n';
  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();

    //Hacer un zig zag para calibrar sin avanzar
    analogWrite(pin_motor_der, calibration_speed - calibration_speed*i/400);
    analogWrite(pin_motor_izq, calibration_speed - calibration_speed*i/400);
    
    if ((i / 10) % 2 == 0) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    }
    else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    }
  }



  //Inicialización de los motores en 0
  analogWrite(pin_motor_izq, 0);
  analogWrite(pin_motor_der, 0);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);   
  delay(2500);
  init_time = 0;
}

//=============================================================
//=============================================================

void loop() {
  init_time = (init_time < 50)? init_time+1: init_time;
  // lecutra sensor izquier y l
  izq = digitalRead(sensor_izq);
  der = digitalRead(sensor_der);

  currState = izq*2 + der;
  // Shift array to right one position and add new value at 0 index
  for (uint8_t i = 1; i < detectionCount; i++) {detectionValues[i] = detectionValues[i - 1];}
  detectionValues[0] = currState;
  // Crear arreglo para contar los valores
  uint8_t count[4] = {};
  // Contar los valores
  for (uint8_t i = 0; i < detectionCount; i++) {count[detectionValues[i]]++;}
  // Determinar el estado con argmax
  uint8_t maxx = 0;
  for (uint8_t i = 1; i < 4; i++) {if (count[i] > count[maxx]) {maxx = i;}}
  detection = maxx;
  // No hay 11
  uint8_t bb = 0;
  for (uint8_t i = 1; i < 4; i++) {if (detectionValues[i] == 3) {bb +=1;}}

  //================= Lista de estados ======================================
  //  0    Detención del robot
  //  1    Navegación normal
  //  2    navegación lenta (en curvas)
  //
  //Serial <<"\nS_Izq: " << izq << '\t' <<"S_der: " << der << '\t' << "state: " << state << "\tDetection: " << detection;
  //Serial <<"\tError: "<< digitalRead(error) << '\t' << "M_izq: "<< digitalRead(m_izq_speed) << '\t' << "M_der: " << digitalRead(m_izq_speed);
  //for (uint8_t i = 0; i < detectionCount; i++){Serial << detectionValues[i] << '\t';}
  if (detection == 2 && init_time==50 && bb==0){ // Criterio de detención
    state = -1;
    int final_iterations = 10;
    while(final_iterations){
      move_robot();
      final_iterations--;
      delay(100);
    }
    analogWrite(pin_motor_izq,0);
    analogWrite(pin_motor_der,0);
    while(1){delay(1000);}
  }
  move_robot();
  delay(100);
}
