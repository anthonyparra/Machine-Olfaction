//---------------------------------------------------------------------------------------------------------------
//                                                  LIBRARIES
//---------------------------------------------------------------------------------------------------------------

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

//---------------------------------------------------------------------------------------------------------------
//                                                  DEFINES
//---------------------------------------------------------------------------------------------------------------

#define anInput     A0  
#define co2Zero     55  
#define led          9

//---------------------------------------------------------------------------------------------------------------
//                                                  LIBRARY CALLS
//---------------------------------------------------------------------------------------------------------------

LiquidCrystal_I2C lcd(0x27, 16, 2);         // Set the LCD address to 0x27 for a 16 chars and 2 line display

//---------------------------------------------------------------------------------------------------------------
//                                                  SETUP
//---------------------------------------------------------------------------------------------------------------
void setup()
{
  Serial.begin(9600);           //Opening Serial port
  
	
	lcd.begin();                  // initialize the LCD
	lcd.backlight();              // Turn on the blacklight and print a message.
	
}

void loop()
{

int co2now[10];                               //int array for co2 readings
int co2raw = 0;                               //int for raw value of co2
int co2comp = 0;                              //int for compensated co2 
int co2ppm = 0;                               //int for calculated ppm
int zzz = 0;                                  //int for averaging
int grafX = 0;                                //int for x value of graph


  //lcd.clear();                     //clear display @ beginning of each loop

  for (int x = 0;x<10;x++){                   //samplpe co2 10x over 2 seconds
    co2now[x]=analogRead(A3);
    delay(200);
  }

for (int x = 0;x<10;x++){                     //add samples together
    zzz=zzz + co2now[x];
    
  }
  co2raw = zzz/10;                            //divide samples by 10
  co2comp = co2raw - co2Zero;                 //get compensated value
  co2ppm = map(co2comp,0,1023,400,5000);      //map value for atmospheric levels

  lcd.setCursor(0,0);                         //set cursor
  lcd.print("CO2 Level:");                    //print title
  lcd.print(co2ppm);                          //print co2 ppm
  lcd.print("PPM");                           //print units
  lcd.setCursor(0,1); 

  
  delay(100);
  if(co2ppm>999){                             //if co2 ppm > 1000
    digitalWrite(led,HIGH);                   //turn on led
  }
  else{                                       //if not
    digitalWrite(led,LOW);                    //turn off led
  }
  
}
