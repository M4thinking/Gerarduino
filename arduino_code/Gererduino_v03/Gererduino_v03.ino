
#include <QTRSensors.h>
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
// Pin de conxión a los motores
int pin_motor_izq = 11;
int pin_motor_der = 10;

#define KD  5  // Constante derivativa PID
#define KP  0.1  // Constante proporcional PID
#define M   50 // Velocidad base de los motores

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

  for (uint16_t i = 0; i < 400; i++){
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  Serial.begin(9600);
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
  delay(1000);
}

void loop() {
  // Generamos la referencia
  int lastError = 0;
  
  // get calibrated sensor values returned in the sensors array, along with the line
  // position, which will range from 0 to 8000, with 1000 corresponding to the line over
  // the middle sensor
  int position = qtr.readLineWhite(sensorValues);
 
  // Error will range from -450000 to +4500.  If we have sensor 0 on the left and sensor 7 on the right  
  int error = position - 4500;
 
  // KP is the a floating-point proportional constant (maybe start with a value around 0.1)
  // KD is the floating-point derivative constant (maybe start with a value around 5)
  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;
 
  // M1 and M2 = M are base motor speeds. 
  // Start with small values for M1 and M2.
  int m_izq_speed = M + motorSpeed;
  int m_der_speed = M - motorSpeed;
 
  // it might help to keep the speeds positive (this is optional)
  // note that you might want to add a similiar line to keep the speeds from exceeding
  // any maximum allowed value
  if (m_izq_speed < 0)
    m_izq_speed = 0;
  if (m_der_speed < 0)
    m_der_speed = 0;
 
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
  
  delay(1);
}
