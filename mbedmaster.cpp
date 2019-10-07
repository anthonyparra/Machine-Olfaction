//  This is sampling code for machine olfaction
//  Anthony Parra
//  08-08-2019


#include "mbed.h"
#include <iostream>
//#include "dataranges.h" unfinished code to see if already mapped smell can be detected

using namespace std;

AnalogIn ain1(p16);
AnalogIn ain2(p17);
AnalogIn ain3(p18);
AnalogIn ain4(p19);
AnalogIn ain5(p20);


int main() {
    printf("VOC, MQ135, MQ8, MQ2, MQ4 \n");
    while(1) {
        unsgined int voc;
        voc = ain1.read();
        printf("%f",voc);
        printf(",");
        printf("%f",ain2.read());
        printf(",");
        printf("%f",ain3.read());
        printf(",");
        printf("%f",ain4.read());
        printf(",");
        printf("%f\n",ain5.read());
        wait(1);
    }
}


//THIS IS FOR DETECTING FEATURE (WIP) 

//bool inRange(unsigned low, unsigned high, unsigned x)
//{
//        return ((x-low) <= (high-low));
//}


//void spiceInRange(){
//    
//    inRange(spiceVOCLower, spiceVOCUpper, voc)? cout << "Yes\n": cout << "No\n";
//
//}



