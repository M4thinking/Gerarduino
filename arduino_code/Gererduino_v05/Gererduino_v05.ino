#include <QTRSensors.h>
QTRSensors qtr;

const uint8_t SensorCount = 6;
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
float KD = 12.5;  // Constante derivativa PID
float KP = 8.0;  // Constante proporcional PID

//=============================================================
//Inicialización de las variables
int lastError = 0, error = 0, state = 0, init_time=0;
int m_izq_speed, m_der_speed, pos, motorSpeed;
int base_speed = 140, max_speed = 160, calibration_speed = 100;

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
  //for (uint8_t i = 0; i < SensorCount; i++){Serial.print(sensorValues[i]);Serial.print('\t');}
//=============================================================
}

//=============================================================
//=============================================================

void setup() {
  Serial.begin(9600);
  // Configuramos los sensores
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {
   A1, A2, A3, A4, A5, A6
  }, SensorCount);
  qtr.setEmitterPin(12);

  // Pines de sensores infrarojo laterales en D7 y D13
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

  analogWrite(pin_motor_izq, 0);
  analogWrite(pin_motor_der, 0);

  delay(500);

  
  // CALIBRAMOS
  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();

    //Hacer un zig zag para calibrar sin avanzar
    analogWrite(pin_motor_der, calibration_speed);
    analogWrite(pin_motor_izq, calibration_speed
    
    );
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
}

//=============================================================
//=============================================================

void loop() {
  init_time = (init_time<20)? init_time+1:init_time;
  //================= Lista de estados ======================================
  //  0    Detención del robot
  //  1    Navegación normal
  //  2    navegación lenta (en curvas)
  //
  Serial <<"S_Izq: " << digitalRead(sensor_izq) << '\t' <<"S_der: " << digitalRead(sensor_der) << '\t' << "state: " << state;
  Serial <<" Error: "<< digitalRead(error) << '\t' << "M_izq: "<< digitalRead(m_izq_speed) << '\t' << "M_der: " << digitalRead(m_izq_speed) << '\n';
  if (state == 0){ // Linea recta
    if (!digitalRead(sensor_izq) && digitalRead(sensor_der)){     // Normal hasta que detecta curva
      base_speed = 110;
      state = 1;
    }
    else if (digitalRead(sensor_izq) && !digitalRead(sensor_der) && init_time == 20){ // Criterio de detención
      analogWrite(pin_motor_izq,0);
      analogWrite(pin_motor_der,0);
      state = -1;
      while(1){delay(1000);};
    }
  }
  else if (state == 1){ // Robot entrando a la curva
    if (digitalRead(sensor_izq) && digitalRead(sensor_der)){    // Lento, se evita bug blanco-blanco
      state = 2;
    }
  }
  else if (state == 2){ // Robot dentro de curva
   if (!digitalRead(sensor_izq) && digitalRead(sensor_der)){     // Salida
      base_speed = 140;
      state = 0;
    }
  }
  move_robot();
  delay(100);
}
