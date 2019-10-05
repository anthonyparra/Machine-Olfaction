#include "mbed.h"
//#include <Map.hpp>
#include <iostream>
#include <math.h>

#define co2Zero 55

using namespace std;

Serial pc(USBTX, USBRX); //tx and rx
AnalogIn   ain(p16);

float map(float in, float inMin, float inMax, float outMin, float outMax) {
  // check it's within the range
  if (inMin<inMax) { 
    if (in <= inMin) 
      return outMin;
    if (in >= inMax)
      return outMax;
  } else {  // cope with input range being backwards.
    if (in >= inMin) 
      return outMin;
    if (in <= inMax)
      return outMax;
  }
  // calculate how far into the range we are
  float scale = (in-inMin)/(inMax-inMin);
  // calculate the output.
  return outMin + scale*(outMax-outMin);
}

int main() {
   while (1) {
    
    float co2now[10];
    float co2raw = 0;
    float co2comp = 0;
    float co2ppm = 0;
    float zzz = 0;
    
        //printf("percentage: %f\n", ain.read());
        //wait(1.0f);
        
        for (int x = 0; x < 10; x++){
            co2now[x] = ain.read();
            wait(0.2f);
        }
        
        for (int x = 0; x < 10; x++){
            zzz = (float) zzz + co2now[x];
            printf("%f", zzz);
            printf("\n");
           
        }
        
        co2raw = (float) zzz / 10;
        printf("co2raw: %f", co2raw);
        printf("\n");
        co2comp = (float) co2raw - (float) co2Zero;
        printf("co2comp: %f", co2comp);
        printf("\n");
        co2ppm = map(co2comp, 0, 1023, 400, 5000);
       
        
        
        printf("CO2 Concentration: ");
        printf("%f", co2ppm);
        printf("PPM");
        printf("\n");
        printf("Analog Input: ");
        printf("%f", ain.read());
        printf("\n");
        
    }
    
}


