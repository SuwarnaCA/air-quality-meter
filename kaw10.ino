#include <SD.h> 
#include <SPI.h>
/*Connect the 5V pin to the 5V pin on the Arduino
Connect the GND pin to the GND pin on the Arduino
Connect CLK to pin 13 or 52
Connect DO to pin 12 or 50
Connect DI to pin 11 or 51
Connect CS to pin 10 or 53*/
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include <dht.h>

dht DHT;
int measurePin = A6;
int ledPower = 12;
LiquidCrystal lcd(10, 11, 5, 4, 3, 2); //change 12 rs to 10
// File myFile;

int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;
 
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;

SoftwareSerial Thermal(8, 9); //Soft RX from printer on D8, soft TX out to printer on D9

int heatTime = 80;
int heatInterval = 255;
char printDensity = 15; 
char printBreakTime = 15;

#define DHT11_PIN 6
/************************Hardware Related Macros************************************/
#define         RL_VALUE                   (5)     //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (9.83)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                   //which is derived from the chart in datasheet
/***********************Software Related Macros************************************/
#define         CALIBARAION_SAMPLE_TIMES    (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
                                                     //normal operation                                                   
/**********************Application Related Macros**********************************/
#define         GAS_LPG                      (0)
#define         GAS_CO                       (1)
#define         GAS_SMOKE                    (2)
#define         GAS_CH4                      (3)
/*****************************Globals***********************************************/
float           LPGCurve[3]  =  {2.3,0.21,-0.47};   //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve. 
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59) 
float           COCurve[3]  =  {2.3,0.72,-0.34};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15) 
float           SmokeCurve[3] ={2.3,0.53,-0.44};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)                                                     
float           CH4Curve[3]  =  {3.3, 0,  -0.38};   //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg2000, lg1), point2: (lg5000,  lg0.7)                                                  
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms
float           Ro1           =  10;                 //Ro is initialized to 10 kilo ohms
float MQ_PIN=8;
float MQ_PIN_6=A1;
int printer=7;
void setup()
{
  
  digitalWrite(42,HIGH);
digitalWrite(43,HIGH);
  digitalWrite(32,HIGH);
digitalWrite(34,HIGH);
digitalWrite(36,HIGH);
digitalWrite(38,HIGH);
digitalWrite(33,LOW);
digitalWrite(35,LOW);
digitalWrite(37,LOW);
digitalWrite(39,LOW);
 
 pinMode(printer,INPUT); //printer button
  pinMode(14,OUTPUT);//green
 pinMode(15,OUTPUT);//yellow
 pinMode(16,OUTPUT);//red
  lcd.begin(20, 4);  
  lcd.setCursor(0, 0);
  lcd.print("Feb  23,2017"); // print out the date

  Serial.begin(9600);  //UART setup, baudrate = 9600bps
  delay(5000);
  lcd.setCursor(0, 0);
   Serial.print("Calibrating sensors...\n"); 
 // lcd.print("Calibrating sensors."); 
  delay(3000);
  lcd.setCursor(0, 0);
  //Ro = MQCalibration(MQ_PIN);                       //Calibracalibration                    
  Serial.print("Device starting up...\n"); //ting the sensor. Please make sure the sensor is in clean air 
   Ro=1.5 ;                                             //when you perform the 
  lcd.print("Device starting up."); 
  
  //Ro1 = MQCalibration1(MQ_PIN_6);                       //Calibrating the sensor. Please make sure the sensor is in clean air 
   Ro1=1.5 ;                
   Serial.println("DHT TEST PROGRAM ");
  
  Serial.println();
  //when you perform the calibration                    
  Serial.print("Ro=");
  Serial.print(Ro); 
  Serial.print("kohm");
  Serial.print("\n");
  Serial.print("Ro1=");
  Serial.print(Ro1);
  Serial.print("kohm");
  Serial.print("\n");
  pinMode(ledPower,OUTPUT);
delay(3000);
lcd.clear();
Thermal.begin(9600); 
}

void initPrinter()
{
 //Modify the print speed and heat
 Thermal.write(27);
 Thermal.write(55);
 Thermal.write(7); //Default 64 dots = 8*('7'+1)
 Thermal.write(heatTime); //Default 80 or 800us
 Thermal.write(heatInterval); //Default 2 or 20us
 //Modify the print density and timeout
 Thermal.write(18);
 Thermal.write(35);
 int printSetting = (printDensity<<4) | printBreakTime;
  Serial.println();
 Serial.println("Printer ready"); 
}
void loop()
{ voMeasured = analogRead(measurePin); // read the dust value
 calcVoltage = voMeasured * (3.3 / 1024);
 dustDensity = 0.17 * calcVoltage - 0.1;
 int chk = DHT.read11(DHT11_PIN);
   float lpg=MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG);
  float co=MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO) ;
  float smoke=MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE);
  float ch4=MQGetGasPercentage1(MQRead(MQ_PIN_6)/Ro,GAS_CH4);
  Serial.print("LPG:"); 
  Serial.print(lpg);
  Serial.print( "ppm" );
   Serial.print("    ");   
   Serial.print("CO:"); 
   Serial.print(co);
   Serial.print( "ppm" );
   Serial.print("    ");
   Serial.print("SMOKE:"); 
   Serial.print(smoke);
   Serial.print( "ppm" );
   Serial.print("    "); 
   Serial.print("  CH4:"); 
   Serial.print(ch4);
   Serial.print( "ppm" );
   Serial.print("    "); 
   Serial.print("Humidity:"); 
   Serial.print(DHT.humidity, 1);
   Serial.print(",\t");
   Serial.print("Temperature:"); 
   Serial.println(DHT.temperature, 1);
   Serial.print(" - Voltage: ");
   Serial.print(calcVoltage);
   Serial.print("-Dust Density: ");
   Serial.println(dustDensity);
   Serial.print("\n");
  lcd.noCursor();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(" "); 
   
   lcd.setCursor(1,0);
   lcd.print("LPG:"); 
   lcd.print(lpg);
   lcd.setCursor(10,0);
   lcd.print(" CO:");
   lcd.print(co);
   lcd.setCursor(0,1);
   lcd.print("SMOKE:"); 
   lcd.print(smoke);
   lcd.setCursor(10, 1); //c r
   lcd.print(" CH4:"); 
   lcd.print(ch4);
   lcd.setCursor(0, 2);
   lcd.print("H&T:"); 
   lcd.print(DHT.humidity, 1);
   lcd.print(" & ");

  lcd.println(DHT.temperature, 1);
 lcd.setCursor(15,2);
  lcd.print("C  ");
 lcd.setCursor(0,3);
  lcd.print("Dust Density: ");
  lcd.print(dustDensity);
  lcd.setCursor(18,3);
 lcd.print("  ");
   delay(2000);
   if ((lpg<=0.18&&  co<=0.05 )|| (smoke<0.06 && ch4<0.001))
  {lcd.clear();
    Serial.print("The Air is clean here!!");
    lcd.setCursor(0,2);
    lcd.print("Air is clean here!");
    digitalWrite(14,HIGH);
  }
   else if (lpg > 0.19  ||co>=0.06  || smoke>0.06  || ch4>0.02)
  {lcd.clear();
    Serial.print(" The polution is in safe limits here!!");
    lcd.setCursor(0,2);
    lcd.print(" The air is slightly");
    lcd.setCursor(0,3);
    lcd.print("polluted");
    digitalWrite(15,HIGH);
  }
  else if (lpg>0.24 || co>0.1|| smoke>0.1 || ch4>0.01)
  {lcd.clear();
    Serial.print(" The Air is polluted!!");
    lcd.setCursor(0,2);
    lcd.print("The Air is highly");
    lcd.setCursor(0,3);
    lcd.print(" polluted!");
        digitalWrite(16,HIGH);
  }
 
 
    if (printer == HIGH)
    {

   Thermal.print(" LPG:"); 
   Thermal.print(lpg);
   Thermal.print("  CO:");
   Thermal.print(co);
   Thermal.println("SMOKE:"); 
   Thermal.print(smoke);
   Thermal.print("  CH4:"); 
   Thermal.print(ch4);
 Thermal.print("H&T:"); 
Thermal.print(DHT.humidity, 1);
   Thermal.print(" & ");

  Thermal.println(DHT.temperature, 1);
 Thermal.print("C");
   Thermal.println("-Dust Density: ");
   Thermal.print(dustDensity);
      }
  }

/****************** MQResistanceCalculation ****************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/ 
float MQResistanceCalculation(float raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}

/***************************** MQCalibration ****************************************
Input:   mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use  
         MQResistanceCalculation to calculates the sensor resistance in clean air 
         and then divides it with RO_CLEAN_AIR_FACTOR_ . RO_CLEAN_AIR_FACTOR_  is about 
         10, which differs slightly between different sensors.
************************************************************************************/ 
float MQCalibration(float mq_pin)
{
  int i;
  float val=0;

  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value

  val = val/RO_CLEAN_AIR_FACTOR ;                        //divided by RO_CLEAN_AIR_FACTOR_co2 yields the Ro 
                                                        //according to the chart in the datasheet 

  return val; 
}

float MQCalibration1(float mq_pin_6)
{
  int i;
  float val1=0;
 
  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val1 += MQResistanceCalculation(analogRead(mq_pin_6));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val1 = val1/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
 
  val1 = val1/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                        //according to the chart in the datasheet 
 
  return val1; 
}
/*****************************  MQRead *********************************************
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/ 
float MQRead(float mq_pin)
{
  int i;
  float rs=0;
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(MQ_PIN));
    delay(READ_SAMPLE_INTERVAL);
  }
  rs = rs/READ_SAMPLE_TIMES;
  return rs;  
}
float MQRead1(float mq_pin_6)
{
  int i;
  float rs1=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs1 += MQResistanceCalculation(analogRead(mq_pin_6));
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs1 = rs1/READ_SAMPLE_TIMES;
 
  return rs1;  
}

/*****************************  MQGetGasPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the target gas
Remarks: This function passes different curves to the MQGetPercentage function which 
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/ 
float MQGetGasPercentage(float rs_ro_ratio, float gas_id)
{
  if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
  }    

  return 0;
}
float MQGetGasPercentage1(float rs_ro_ratio1, float gas_id1)
{
   if ( gas_id1 == GAS_CH4 ) {
      return MQGetPercentage(rs_ro_ratio1,CH4Curve);
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
float  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}
float  MQGetPercentage1(float rs_ro_ratio1, float *pcurve1)
{
  return (pow(10, (((log(rs_ro_ratio1)-pcurve1[1])/pcurve1[2]) + pcurve1[0])));
}
  



