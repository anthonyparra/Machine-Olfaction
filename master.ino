// Program: I2C master reader for Machine Olfaction
// Programmer: Anthony Parra (anthonyparra.github.io)
// Date: July 31, 2014

#include <Wire.h>
#include <MQ2.h>
#include <LiquidCrystal_I2C.h>
#include <AltSoftSerial.h>

#define PAYLOAD_SIZE 7 // how many bytes to expect from each I2C salve node
#define NODE_MAX 6 // maximum number of slave nodes (I2C addresses) to probe
#define START_NODE 2 // The starting I2C address of slave nodes
#define NODE_READ_DELAY 1000 // Some delay between I2C node reads
#define co2Zero     55  
#define led          9
#define         MQ_PIN                       (A1)    //MQ-8 Sensor
#define         RL_VALUE                     (10)    //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (9.21)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
                                                     //normal operation                                                    //which is derived from the chart in datasheet
#define         GAS_H2                      (A1)     //MQ-8 Sensor

LiquidCrystal_I2C lcd(0x27, 16, 2);
AltSoftSerial BTserial;

char c=' ';
boolean NL = true;
 
int nodePayload[PAYLOAD_SIZE];
float           H2Curve[3]  =  {2.3, 0.93,-1.44};
float           Ro           =  10;

int Analog_Input = A0; //MQ-2 Sensor
int lpg, co, smoke;

const int gasPin = A2; //MQ-4 Sensor

MQ2 mq2(Analog_Input);

void setup()
{
  Serial.begin(9600);  
  BTserial.begin(9600);
  Serial.println("This is the Master Controller");
  BTserial.write("This is the Master Controller");
  lcd.begin();                  // initialize the LCD
  lcd.backlight();              // Turn on the blacklight and print a message.
  lcd.print("Calibrating...");
  BTserial.write("Calibrating...");

  Serial.print("Calibrating...\n");                
  Ro = MQCalibration(MQ_PIN);                        //Calibrating the sensor. Please make sure the sensor is in clean air 
                                                     //when you perform the calibration                    
  Serial.print("Calibration is done...\n"); 
  BTserial.write("Calibration is done...");
  Serial.print("Ro=");
  Serial.print(Ro);
  Serial.print("kohm");
  Serial.print("\n");

  mq2.begin();

  lcd.clear();

  Serial.println("Temperature, Humidity, Temperature, VOC, CO2, H2, LPG, CO, SMOKE, METHANE,");
  BTserial.write("Temperature, Humidity, Temperature, VOC, CO2, H2, LPG, CO, SMOKE, METHANE,");
  Wire.begin();        // Activate I2C link
}

void loop()
{
  
  for (int nodeAddress = START_NODE; nodeAddress <= NODE_MAX; nodeAddress++) { // we are starting from Node address 2
    Wire.requestFrom(nodeAddress, PAYLOAD_SIZE);    // request data from node#
    if(Wire.available() == PAYLOAD_SIZE) {  // if data size is avaliable from nodes
      //for (int i = 0; i < PAYLOAD_SIZE; i++) nodePayload[i] = Wire.read();  // get nodes data
      //for (int j = 0; j < PAYLOAD_SIZE; j++) Serial.print(nodePayload[j]);  // print nodes data 
      Serial.print(","); 
      BTserial.write(",");
      nodePayload[0] = Wire.read();
      Serial.print(nodePayload[0]);
      BTserial.write(nodePayload[0]);
      nodePayload[1] = Wire.read();
      Serial.print(",");
      BTserial.write(",");
      nodePayload[2] = Wire.read();
      Serial.print(nodePayload[2]);
      BTserial.write(nodePayload[2]);
      nodePayload[3] = Wire.read();
      Serial.print(",");
      nodePayload[4] = Wire.read();
      Serial.print(nodePayload[4]);
      BTserial.write(nodePayload[4]);
      Serial.print(",");     
      BTserial.write(","); 
      nodePayload[6] = Wire.read();
      Serial.print(nodePayload[6]);  
      BTserial.write(nodePayload[6]);
      Serial.print(",");  
      BTserial.write(",");

      //CO2 Sensor
      int co2now[10];                               //int array for co2 readings
      int co2raw = 0;                               //int for raw value of co2
      int co2comp = 0;                              //int for compensated co2 
      int co2ppm = 0;                               //int for calculated ppm
      int zzz = 0;                                  //int for averaging
      int grafX = 0;                                //int for x value of graph

      for (int x = 0;x<10;x++){                   //samplpe co2 10x over 2 seconds
        co2now[x]=analogRead(A3);                 //MQ-135 sensor
        delay(200);
      }

      for (int x = 0;x<10;x++){                     //add samples together
        zzz=zzz + co2now[x];
    
      }
      co2raw = zzz/10;                            //divide samples by 10
      co2comp = co2raw - co2Zero;                 //get compensated value
      co2ppm = map(co2comp,0,1023,400,5000);      //map value for atmospheric levels
      
      lcd.setCursor(0,0);                         //set cursor
      Serial.print(co2ppm);
      BTserial.write(co2ppm);
      Serial.print(",");
      BTserial.write(",");

      //H2 Sensor
      Serial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_H2) );
      BTserial.write(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_H2) );
      Serial.print(",");
      BTserial.write(",");

      //LPG, CO, and SMOKE Sensor

      float* values= mq2.read(false); //set it false if you don't want to print the values in the Serial
      //lpg = values[0];
      lpg = mq2.readLPG();
      //co = values[1];
      co = mq2.readCO();
      //smoke = values[2];
      smoke = mq2.readSmoke();

      Serial.print(lpg);
      BTserial.write(lpg);
      Serial.print(",");
      BTserial.write(",");
      Serial.print(co);
      BTserial.write(co);
      Serial.print(",");
      BTserial.write(",");
      Serial.print(smoke);
      BTserial.write(smoke);
      Serial.print(",");
      BTserial.write(",");

      //Methane Sensor

      Serial.print(analogRead(gasPin));
      BTserial.write(analogRead(gasPin));
      Serial.println(",");
      BTserial.write(",\n");
      
      delay(1000);
      }
  
    }
    
}

/****************** MQResistanceCalculation ****************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/ 
float MQResistanceCalculation(int raw_adc) {
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}
 
/***************************** MQCalibration ****************************************
Input:   mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use  
         MQResistanceCalculation to calculates the sensor resistance in clean air 
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about 
         10, which differs slightly between different sensors.
************************************************************************************/ 
float MQCalibration(int mq_pin) {
  int i;
  float val=0;
 
  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
 
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                        //according to the chart in the datasheet 
 
  return val; 
}
/*****************************  MQRead *********************************************
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/ 
float MQRead(int mq_pin) {
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}
 
/*****************************  MQGetGasPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the target gas
Remarks: This function passes different curves to the MQGetPercentage function which 
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/ 
int MQGetGasPercentage(float rs_ro_ratio, int gas_id) {
  if ( gas_id == GAS_H2) {
     return MQGetPercentage(rs_ro_ratio,H2Curve);
  }  
  return 0;
}
 
/*****************************  MQGetPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm) 
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a 
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic 
         value.
************************************************************************************/ 
int  MQGetPercentage(float rs_ro_ratio, float *pcurve) {
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}
