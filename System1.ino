#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include "SparkFunHTU21D.h"
#include <stdlib.h>

#define BMP_SCK  13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS   10
#define co2Zero     55  

//Create an instance of the object
HTU21D myHumidity;

Adafruit_BMP280 bmp(BMP_CS); // hardware SPI

void setup() {
  
  Serial.begin(9600);
  Wire.begin(); 
  
  Serial.println("Readings: ");
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

  Serial.print("Temperature:");
  char sendTempLabel[] = "Temperature:";
  Serial.write(sendTempLabel, 15);
  Serial.print(temp, 1);
  char sendTemp[5];
  dtostrf(temp, 6, 2, sendTemp);
  Serial.write(sendTemp, sizeof(sendTemp));
  Serial.println("C");
  char sendTempUnit[] = "C";
  Serial.write(sendTempUnit, 3);
  Serial.print("Humidity:");
  char sendHumdLabel[] = "Humidity:";
  Serial.write(sendHumdLabel, 10);
  Serial.print(humd, 1);
  char sendHumd[5];
  dtostrf(humd, 6, 2, sendHumd);
  Serial.write(sendHumd, sizeof(sendHumd));
  Serial.println("%");
  char sendHumdUnit[] = "%";
  Serial.write(sendTempLabel, 3);
  Serial.println();

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
  int reading = analogRead(A2);
  Serial.print(F("VOC PPM:"));
  Serial.println(reading);
  //delay(3000);

  //MQ VOC
  
  int co2now[10];                               //int array for co2 readings
int co2raw = 0;                               //int for raw value of co2
int co2comp = 0;                              //int for compensated co2 
int co2ppm = 0;                               //int for calculated ppm
int zzz = 0;                                  //int for averaging
int grafX = 0;                                //int for x value of graph


  //lcd.clear();                     //clear display @ beginning of each loop

  for (int x = 0;x<10;x++){                   //samplpe co2 10x over 2 seconds
    co2now[x]=analogRead(A3);
    delay(20);
  }

for (int x = 0;x<10;x++){                     //add samples together
    zzz=zzz + co2now[x];
    
  }
  co2raw = zzz/10;                            //divide samples by 10
  co2comp = co2raw - co2Zero;                 //get compensated value
  co2ppm = map(co2comp,0,1023,400,5000);      //map value for atmospheric levels
                      
  Serial.print("CO2 Level:");                    //print title
  Serial.print(co2ppm);                          //print co2 ppm
  Serial.print("PPM");                           //print units

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
