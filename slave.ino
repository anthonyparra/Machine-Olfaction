// Program: I2C slave sender template for multi-node Arduino I2C network
// Programmer: Hazim Bitar (techbitar.com)
// Date: March 30, 2014
// This example code is in the public domain.

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include "SparkFunHTU21D.h"

#define NODE_ADDRESS 2  // Change this unique address for each I2C slave node
#define PAYLOAD_SIZE 7 // Number of bytes  expected to be received by the master I2C node

#define BMP_SCK  13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS   10

Adafruit_BMP280 bmp(BMP_CS); // hardware SPI

byte nodePayload[PAYLOAD_SIZE];

const int gasSensor = A1; 
int sensorValue;

//Create an instance of the object
HTU21D myHumidity;

void setup()
{

  Serial.begin(9600);  
  Serial.println("This is the Slave Controller");
  Serial.println("Temperature, Humidity, Temperature, Pressure, VOC, MQVOC");
  myHumidity.begin();

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

   bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  
  Serial.println("SLAVE SENDER NODE");
  Serial.print("Node address: ");
  Serial.println(NODE_ADDRESS);
  Serial.print("Payload size: ");
  Serial.println(PAYLOAD_SIZE);
  Serial.println("***********************");

  Wire.begin(NODE_ADDRESS);  // Activate I2C network
  Wire.onRequest(requestEvent); // Request attention of master node
}

void loop()
{ 

  //Humidity, Temp, Time
  float humd = myHumidity.readHumidity();
  float temp = myHumidity.readTemperature();
  char comma;
  comma = ',';

  Serial.print(temp);
  nodePayload[0] = temp; // Sending Temp to master
  Serial.print(",");
  nodePayload[1] = comma;
  Serial.print(humd);
  nodePayload[2] = humd; // Sending Humd to master
  Serial.print(",");
  nodePayload[3] = comma;

  //Temp, Pressure, Altitude
  Serial.print(bmp.readTemperature());
  nodePayload[4] = bmp.readTemperature(); // Sending Temp to master
  Serial.print(",");
  Serial.print(bmp.readPressure());
  float press = bmp.readPressure();
  nodePayload[5] = press; // Sending Press to master
  Serial.print(",");

  //VOC
  int reading = analogRead(A2);
  Serial.println(reading);
  nodePayload[6] = reading; // Sending Press to master
  Serial.print(",");
  
}

void requestEvent()
{
  Wire.write(nodePayload,PAYLOAD_SIZE);   
}
