
#include <QTRSensors.h>
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
// Pin de conxión a los motores
int pin_motor_izq = 11;
int pin_motor_der = 10;

#define KD 2  // Constante derivativa PID
#define KP 0.05  // Constante proporcional PID
#define base_speed 100 // Velocidad base de los motores
#define max_speed 150 // Velocidad máxima de los motores
template <typename T>
Print& operator<<(Print& printer, T value){
  printer.print(value);
  return printer;
}

void pprint(char* nombre, int valor){
  Serial << nombre << valor << '\t';
}

void setup()
{
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5,A6, A7}, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  Serial.begin(9600);
  
  analogWrite(pin_motor_izq,0);
  analogWrite(pin_motor_der,base_speed);
  Serial.println("Calibrando");
  for (uint16_t i = 0; i < 400; i++){
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  
  analogWrite(pin_motor_izq,0);
  analogWrite(pin_motor_der,0);
  delay(1000);
  
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++){
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(200);
}
void loop() {
  // Generamos la referencia
  int lastError = 0;
  
  // get calibrated sensor values returned in the sensors array, along with the line
  // position, which will range from 0 to 8000, with 1000 corresponding to the line over
  // the middle sensor
  int position = qtr.readLineWhite(sensorValues);
 
  // Error will range from -4500 to +4500.  If we have sensor 0 on the left and sensor 7 on the right  
  int error = (position - 4500)/100;
 
  // KP is the a floating-point proportional constant (maybe start with a value around 0.1)
  // KD is the floating-point derivative constant (maybe start with a value around 5)
  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;
  pprint("Aditional speed: ", motorSpeed);
  // M1 and M2 = M are base motor speeds. 
  // Start with small values for M1 and M2.
  int m_izq_speed = base_speed + motorSpeed;
  int m_der_speed = base_speed - motorSpeed;
 
  // it might help to keep the speeds positive (this is optional)
  // note that you might want to add a similiar line to keep the speeds from exceeding
  // any maximum allowed value
  if (m_izq_speed < 0){
    pprint("a: ", m_izq_speed);
    m_izq_speed = 0;
  }
  if (m_der_speed < 0){
    pprint("b: ", m_der_speed);
    m_der_speed = 0;
  }
    
  // Clipping de la velocidad
  if (max_speed < m_izq_speed){
    m_izq_speed = max_speed;
  }
  if (max_speed < m_der_speed){
    m_der_speed = max_speed;
  }
 
  // set motor speeds using the two motor speed variables above
  analogWrite(pin_motor_izq,m_izq_speed);
  analogWrite(pin_motor_der,m_der_speed);
  
  Serial << "Ref: " << error << '\t' << '\t';
  pprint("Motor izq: ", m_izq_speed);
  pprint("Motor der: ", m_der_speed);
  Serial.println();
  
  // Mostrar valores medidos por los sensores
  //for (uint8_t i = 0; i < SensorCount; i++){
  //  Serial.print(sensorValues[i]);
  //  Serial.print('\t');
  //}
  //Serial.println(position);
}
