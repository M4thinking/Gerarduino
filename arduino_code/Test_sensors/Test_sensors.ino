#include <QTRSensors.h>
QTRSensors qtr;

int sensor_izq = 13, sensor_der = 7;
int pos, der, izq;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5,A6, A7}, SensorCount);
  qtr.setEmitterPin(12);

  // Pines de sensores infrarojo laterales en D7 y D13
  pinMode(sensor_izq, INPUT);
  pinMode(sensor_der, INPUT);

  Serial.println("Calibrando...");
  for (uint16_t i   = 0; i < 100; i++){
    qtr.calibrate();
    }
  
  Serial.print('\n');
}



void loop() {
  // put your main code here, to run repeatedly:
  pos = qtr.readLineWhite(sensorValues);
  der = digitalRead(sensor_der);
  izq = digitalRead(sensor_izq);

  // Printeamos
  for (uint8_t i = 0; i < SensorCount; i++){
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.print("\t");
  Serial.print(izq);
  Serial.print('\t');
  Serial.print(der);
  Serial.print('\n');
  delay(100);
}
