#include <QTRSensors.h>
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

//==============================================================
// Inicializamos los pines
int pin_motor_izq = 6, pin_motor_der = 3;
int sensor_izq = 7, sensor_der = 13;
int enable;

const int IN1 = 4;
const int IN2 = 5;
const int IN3 = 11;
const int IN4 = 10;

//==============================================================
// Constantes del PD
#define KD 2  // Constante derivativa PID
#define KP 1  // Constante proporcional PID

//=============================================================
//Inicialización de las variables
int lastError = 0, error = 0, state = 0;
int m_izq_speed, m_der_speed, pos, motorSpeed;
int base_speed = 150, max_speed = 160;

//=============================================================
// Función que mueve al robot
void move_robot(){
  // Leer posición
  pos = qtr.readLineWhite(sensorValues);
 
  // Mapear la posición a un valor entre -100 y 100
  error = map(pos, 0, 8000, -100, 100);
  // Limitar la velocidad de los motores
  
  // Acción de control
  motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;
 
  // Limit the motor speeds to the range 0 to 255
  m_izq_speed = constrain(base_speed + motorSpeed, 0, max_speed);
  m_der_speed = constrain(base_speed - motorSpeed, 0, max_speed);
 
  // Hechamos a andar
  analogWrite(pin_motor_izq,m_izq_speed);
  analogWrite(pin_motor_der,m_der_speed);
  }

//=============================================================
//=============================================================

void setup(){
  // Configuramos los sensores
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5,A6, A7}, SensorCount);
  qtr.setEmitterPin(12);

  // Pines de sensores infrarojo laterales en D7 y D13
  //pinMode(sensor_izq, INPUT);
  //pinMode(sensor_der, INPUT);
  pinMode(13,OUTPUT);
  
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
  
  delay(500);
  
  // CALIBRAMOS
  for (uint16_t i = 0; i < 200; i++){
    qtr.calibrate();
      
    //Hacer un zig zag para calibrar sin avanzar
    analogWrite(pin_motor_der,base_speed);
    analogWrite(pin_motor_izq, 0);
    if ((i/20)%2 == 0){
      digitalWrite(13,HIGH);
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2,LOW);
    }
    else{
      digitalWrite(13,LOW);
      digitalWrite(IN1,LOW);
      digitalWrite(IN2,HIGH);
      }
  }


  
  //Inicialización de los motores en 0
  analogWrite(pin_motor_izq,0);
  analogWrite(pin_motor_der,0);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2,LOW);
  delay(1000);
}

//=============================================================
//=============================================================

void loop() {
//  if (state == 0){ // condición de detención
//    if (digitalRead(sensor_der) && !digitalRead(sensor_izq) && digitalRead(enable) ){state = 1;}
//    else{delay(100);}
//    }

//  else if (state == 1){// robot andando
//    if (digitalRead(sensor_der) && !digitalRead(sensor_izq)){ // Verificamos si debemos detener
  //    state = 0;
//      delay(1000)
//      base_speed = 0;
//      move_robot();
//      delay(10000);
//      }
//     else{
//      move_robot();
//      delay(10);
//      }
//    }

move_robot();
delay(100);

//================= Lista de estados ======================================
//  0    Detención del robot
//  1    Navegación normal
//  2    navegación lenta (en curvas) 
//
//  if (state == 0){ // Linea recta
//    if (!digitalRead(sensor_izq) && digitalRead(sensor_der)){     // Normal hasta que detecta curva
//      base_speed = 80;
//      state = 1;
//    }
//    else if (digitalRead(sensor_izq) && !digitalRead(sensor_der)){ // Criterio de detención
//      analogWrite(pin_motor_izq,0);
//      analogWrite(pin_motor_der,0);
//      state = -1;
//      while(1){delay(1000);};
//    }
//  }
//  else if (state == 1){ // Robot entrando a la curva
//    if (!digitalRead(sensor_izq) && !digitalRead(sensor_der)){    // Lento, se evita bug blanco-blanco
//      state = 2;
//    }
//  }
//  else if (state == 2){ // Robot dentro de curva
//   if (!digitalRead(sensor_izq) && digitalRead(sensor_der)){     // Salida
//      base_speed = 100;
//      state = 0;
//    }
//  }
}
