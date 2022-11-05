#include <QTRSensors.h>
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// Pin de conxión a los motores
int pin_motor_izq = 2, pin_motor_der = 3;
int sensor_izq = 7, sensor_der = 13;

const int IN1 = 4;
const int IN2 = 5;
const int IN3 = 0;
const int IN4 = 1;
const int ENA = 3;
const int ENB = 2;

#define KD 2  // Constante derivativa PID
#define KP 0.01  // Constante proporcional PID

//Inicialización de las variables
int lastError = 0, error = 0, state = 0;
int m_izq_speed, m_der_speed, pos, motorSpeed;
int base_speed = 100, max_speed = 250;


template <typename T>
Print& operator<<(Print& printer, T value){
  printer.print(value);
  return printer;
}

void pprint(const char* nombre, int valor){
  //Serial << nombre << valor << '\n';
}

void move_robot(){
   // get calibrated sensor values returned in the sensors array, along with the line
  // position, which will range from 0 to 8000, with 1000 corresponding to the line over
  // the middle sensor
  pos = qtr.readLineWhite(sensorValues);
 
  // Mapear la posición a un valor entre -100 y 100
  error = map(pos, 0, 8000, -100, 100);
  // Limitar la velocidad de los motores
  //pprint("Error: ", error);
 
  // KP is the a floating-point proportional constant (maybe start with a value around 0.1)
  // KD is the floating-point derivative constant (maybe start with a value around 5)
  motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;
  //pprint("Aditional speed: ", motorSpeed);
 
  // Limit the motor speeds to the range 0 to 255
  m_izq_speed = constrain(base_speed + motorSpeed, 0, max_speed);
  m_der_speed = constrain(base_speed - motorSpeed, 0, max_speed);
 
  // set motor speeds using the two motor speed variables above
  analogWrite(pin_motor_izq,m_izq_speed);
  analogWrite(pin_motor_der,m_der_speed);
  
  //Serial << "Ref: " << error << '\t' << '\t';
  //pprint("Motor izq: ", m_izq_speed);
  //pprint("Motor der: ", m_der_speed);
  }

void setup()
{
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5,A6, A7}, SensorCount);
  qtr.setEmitterPin(12);

  // Pines de sensores infrarojo laterales en D7 y D13
  pinMode(sensor_izq, INPUT);
  pinMode(sensor_der, INPUT);
  // Pines de sentido de referencia motor derecho
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  // Pines de sentido de referencia motor izquierdo
  //pinMode(IN3, OUTPUT);
  //pinMode(IN4, OUTPUT);
  //digitalWrite(IN3, LOW);
  //digitalWrite(IN4, HIGH);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  //Serial.begin(9600);
  //Serial.println("Calibrando...");
  for (uint16_t i = 0; i < 400; i++){
    qtr.calibrate();
    //Hacer un zig zag para calibrar sin avanzar
    if (i % 50 == 0){
      analogWrite(pin_motor_izq,50);
      analogWrite(pin_motor_der,0);
    }
    else if (i % 50 == 50){
      analogWrite(pin_motor_izq,50);
      analogWrite(pin_motor_der,0);
    }
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  
  //Inicialización de los motores en 0
  //Serial.println("Inicializando motores...");
  analogWrite(pin_motor_izq,0);
  analogWrite(pin_motor_der,0);
  delay(1000);
  
  //Serial.println();
  //Serial.println();
  delay(200);
}


void loop() {
  if (state == 0){ // Linea recta
    if (!digitalRead(sensor_izq) && digitalRead(sensor_der)){     // Normal hasta que detecta curva
      base_speed = 80;
      state = 1;
    }
    else if (digitalRead(sensor_izq) && !digitalRead(sensor_der)){ // Criterio de detención
      analogWrite(pin_motor_izq,0);
      analogWrite(pin_motor_der,0);
      state = -1;
      while(1){delay(1000);};
    }
  }
  else if (state == 1){ // Robot entrando a la curva
    if (!digitalRead(sensor_izq) && !digitalRead(sensor_der)){    // Lento, se evita bug blanco-blanco
      state = 2;
    }
  }
  else if (state == 2){ // Robot dentro de curva
    if (!digitalRead(sensor_izq) && digitalRead(sensor_der)){     // Salida
      base_speed = 100;
      state = 0;
    }
  }
  move_robot();
  
  // Mostrar valores medidos por los sensores
  //for (uint8_t i = 0; i < SensorCount; i++){
  //  Serial.print(sensorValues[i]);
  //  Serial.print('\t');
  //}
  //Serial.println(position);
  delay(500);
}
