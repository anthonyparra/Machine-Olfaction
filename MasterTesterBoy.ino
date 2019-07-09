#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK  13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS   10
#include "SparkFunHTU21D.h"

//Create an instance of the object
HTU21D myHumidity;

Adafruit_BMP280 bmp(BMP_CS); // hardware SPI

const int gasSensor = A1; 
int sensorValue;

void setup() {
  
  Serial.begin(9600);
  Serial.println("Demo Readings: ");
  myHumidity.begin();

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
   
}

void loop() {

  //Humidity, Temp, Time
  float humd = myHumidity.readHumidity();
  float temp = myHumidity.readTemperature();

  Serial.print("Time:");
  Serial.print(millis());
  Serial.print(" Temperature:");
  Serial.print(temp, 1);
  Serial.print("C");
  Serial.print(" Humidity:");
  Serial.print(humd, 1);
  Serial.print("%");

  Serial.println();
  //delay(3000);

  //Temp, Pressure, Altitude
  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
  Serial.println(" m");

  //delay(3000);

  //VOC
  int reading = analogRead(A0);
  Serial.print(F("VOC PPM:"));
  Serial.println(reading);
  //delay(3000);

  //MQ VOC
  
  /*float voltage;
  voltage = getVoltage(gasSensor);

  Serial.print(F("MQVOC Voltage:"));
  Serial.println(voltage);
  delay(5000); */

  sensorValue = analogRead(1);       // read analog input pin 0
  Serial.print("AirQuality=");
  Serial.print(sensorValue, DEC);               // prints the value read
  Serial.println(" PPM");

  Serial.println();
  delay(3000);
  
}

float getVoltage(int pin)
{
return (analogRead(pin) * 0.004882814);
  // This equation converts the 0 to 1023 value that analogRead()
  // returns, into a 0.0 to 5.0 value that is the true voltage
  // being read at that pin.
 
}
