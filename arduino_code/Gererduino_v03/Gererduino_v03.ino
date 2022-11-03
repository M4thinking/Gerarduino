#include <QTRSensors.h>
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
// Pin de conxión a los motores
int pin_motor_izq = 2, pin_motor_der = 3;

#define KD 2  // Constante derivativa PID
#define KP 0.01  // Constante proporcional PID
#define base_speed 100 // Velocidad base de los motores
#define max_speed 150 // Velocidad máxima de los motores

//Inicialización de las variables
int lastError = 0, error = 0, d13_state = 0;
int m_izq_speed, m_der_speed, position;


template <typename T>
Print& operator<<(Print& printer, T value){
  printer.print(value);
  return printer;
}

void pprint(char* nombre, int valor, char* end){
  Serial << nombre << valor << end;
}

void setup()
{
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5,A6, A7}, SensorCount);
  qtr.setEmitterPin(2);

  // Pines de sensores infrarojo laterales en D7 y D13
  pinMode(7, INPUT);
  pinMode(13, INPUT);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  Serial.begin(9600);
  Serial.println("Calibrando...");
  for (uint16_t i = 0; i < 400; i++){
    qtr.calibrate();
    //Hacer un zig zag para calibrar sin avanzar
    if (i % 100 == 0){
      analogWrite(pin_motor_izq,50);
      analogWrite(pin_motor_der,0);
    }
    else if (i % 100 == 50){
      analogWrite(pin_motor_izq,0);
      analogWrite(pin_motor_der,50);
    }
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  
  //Inicialización de los motores en 0
  Serial.println("Inicializando motores...");
  analogWrite(pin_motor_izq,0);
  analogWrite(pin_motor_der,0);
  delay(1000);
  
  for (uint8_t i = 0; i < SensorCount; i++){
    pprint('', qtr.calibrationOn.minimumOn[i], ' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++){
    pprint('', qtr.calibrationOn.maximum[i], ' ');
  }
  Serial.println();
  Serial.println();
  delay(200);
}


void loop() {
  // Si D7 detecta blanco, se detiene para siempre
  if (digitalRead(7) == HIGH && digitalRead(13) == LOW){
    analogWrite(pin_motor_izq,0);
    analogWrite(pin_motor_der,0);
    while(1);
  }
  // // Si D13 es HIGH y antes no. Baja la velocidad de los motores (Entra en curva)
  // if (digitalRead(13) == HIGH && d13_state == 0){
  //   d13_state = 1;
  //   base_speed = 80;
  // }
  // // Si D13 es HIGH y antes era HIGH. Sube la velocidad de los motores (Sale de curva)
  // else if (digitalRead(13) == HIGH && d13_state == 1){
  //   d13_state = 0;
  //   base_speed = 100;
  // }
  
  // get calibrated sensor values returned in the sensors array, along with the line
  // position, which will range from 0 to 8000, with 1000 corresponding to the line over
  // the middle sensor
  position = qtr.readLineWhite(sensorValues);
 
  // Mapear la posición a un valor entre -100 y 100
  error = map(position, 0, 8000, -100, 100);
  // Limitar la velocidad de los motores
  pprint("Error: ", error);
 
  // KP is the a floating-point proportional constant (maybe start with a value around 0.1)
  // KD is the floating-point derivative constant (maybe start with a value around 5)
  motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;
  pprint("Aditional speed: ", motorSpeed);
 
  // Limit the motor speeds to the range 0 to 255
  m_izq_speed = constrain(base_speed + motorSpeed, 0, max_speed);
  m_der_speed = constrain(base_speed - motorSpeed, 0, max_speed);
 
  // set motor speeds using the two motor speed variables above
  analogWrite(pin_motor_izq,m_izq_speed);
  analogWrite(pin_motor_der,m_der_speed);
  
  Serial << "Ref: " << error << '\t' << '\t';
  pprint("Motor izq: ", m_izq_speed);
  pprint("Motor der: ", m_der_speed);
  
  // Mostrar valores medidos por los sensores
  //for (uint8_t i = 0; i < SensorCount; i++){
  //  Serial.print(sensorValues[i]);
  //  Serial.print('\t');
  //}
  //Serial.println(position);
  delay(5);
}
