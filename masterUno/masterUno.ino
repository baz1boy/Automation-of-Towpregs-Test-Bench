#include <Wire.h>
#include <HX711_ADC.h>
#include <EEPROM.h>

#define ENCODER_A_PIN 2 // green
#define ENCODER_Z_PIN 3 // blue 
#define ENCODER_B_PIN 4 // grey

#define slaveAddress 0x08

//pins:
const int HX711_dout_1 = 8; 
const int HX711_sck_1 = 9; 
const int HX711_dout_2 = 11; 
const int HX711_sck_2 = 12; 

//HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell_1(HX711_dout_1, HX711_sck_1); //HX711 1
HX711_ADC LoadCell_2(HX711_dout_2, HX711_sck_2); //HX711 2

const int calVal_eepromAdress_1 = 0; // eeprom adress for calibration value load cell 1 (4 bytes)
const int calVal_eepromAdress_2 = 4; // eeprom adress for calibration value load cell 2 (4 bytes)
unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
const double serialPrintInterval = 50; //increase value to slow down serial print activity
unsigned long t_LC = 0;
bool onoff_force = false;
double timer_LC = 0.0;

// Incremental Encoder
volatile long temp, counter = 0;
double timer_IE = 0.0;
unsigned long t_IE = 0;
bool onoff_angle = false;
double anglechange;
double angleset;

//
bool onoff_test = false;


//Cmd
char cmd;
int cmdint;

void setup() {
  Serial.begin(57600);

  //------------------------- Incremental Encoder --------------------
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);// Set Interrupt Pin in PULL-UP Mode
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);// Set Pin in PULL-UP Mode
  pinMode(ENCODER_Z_PIN, INPUT_PULLUP);// Set Interrupt Pin in PULL-UP Mode

  // Use attachInterrupt function to set interrupt pin, executing function when triggered, and mode
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), interrupt_a_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_Z_PIN), interrupt_z_change, CHANGE);

  //------------------------- Load Cell --------------------------------
  float calibrationValue_1; // calibration value load cell 1
  float calibrationValue_2; // calibration value load cell 2

  EEPROM.get(calVal_eepromAdress_1, calibrationValue_1); // uncomment this if you want to fetch the value from eeprom
  EEPROM.get(calVal_eepromAdress_2, calibrationValue_2); // uncomment this if you want to fetch the value from eeprom

  LoadCell_1.begin();
  LoadCell_2.begin();

  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  byte loadcell_1_rdy = 0;
  byte loadcell_2_rdy = 0;
  while ((loadcell_1_rdy + loadcell_2_rdy) < 2) { //run startup, stabilization and tare, both modules simultaniously
    if (!loadcell_1_rdy) loadcell_1_rdy = LoadCell_1.startMultiple(stabilizingtime, _tare);
    if (!loadcell_2_rdy) loadcell_2_rdy = LoadCell_2.startMultiple(stabilizingtime, _tare);
  }
  
  LoadCell_1.setCalFactor(calibrationValue_1); // user set calibration value (float)
  LoadCell_2.setCalFactor(calibrationValue_2); // user set calibration value (float)
  
  //Serial.println("Startup is complete");
  //-------------------------- I2C ----------------------------------
  Wire.begin();
}

void loop() 
{
  if (Serial.available()) {      
    cmd = Serial.read();           
    cmdint = Serial.parseInt();

    runUsercmd();  
  }

  if(onoff_angle && !onoff_force){
    angleMeasure();
  }else if(onoff_force && !onoff_angle){
    forceMeasure();
  }else if(onoff_force && onoff_angle){
    angleMeasure();
    forceMeasure();
  }
  
  if(onoff_test){
    testMeasure();
  }
}

void testMeasure(){
  static boolean newDataReady = 0;
  // check for new data/start next conversion:
  if (LoadCell_1.update()) newDataReady = true;
  LoadCell_2.update();

  //get smoothed value from data set
  if ((newDataReady)) {
    if (millis() > t_LC + serialPrintInterval) {
      float a = LoadCell_1.getData();
      float b = LoadCell_2.getData();
      anglechange = 0.36 * counter / 2;
      
      Serial.print("T");
      Serial.print(timer_LC); 
      Serial.print('\t');
      Serial.print(anglechange);
      Serial.print('\t');
      Serial.print(a);
      Serial.print('\t');
      Serial.println(b);

      String i2c = String(anglechange) + ',' + String(a) + ',' + String(b) + ',';
      //Serial.println(i2c);
      Wire.beginTransmission(slaveAddress); 
      Wire.write(i2c.c_str());
      Wire.endTransmission();
      //Wire.beginTransmission(2);
      //Wire.write(String(angle).c_str());
      //Wire.endTransmission();
      
      newDataReady = 0;
      t_LC = millis();
      timer_LC = timer_LC + 0.050;
    }
  } 
}

void angleMeasure(){
  if (millis() > t_IE + serialPrintInterval) {
     anglechange = 0.36 * counter / 2;
     Serial.print("A");
     Serial.print(timer_IE);
     Serial.print('\t');
     Serial.println(anglechange);
    
     t_IE = millis();
     timer_IE = timer_IE + 0.050;
  } 
}

// 引脚A中断时，调用的函数
void interrupt_a_change() 
{
  int temp_a = digitalRead(ENCODER_A_PIN);
  int temp_b = digitalRead(ENCODER_B_PIN);

  // 判断编码器的选择方向
  //if(onoff_angle == 1){
    if (temp_a == temp_b)
    {
      counter--; // 
    } else {
    counter++; // 
    }
  //}
}

// 引脚Z中断时，调用的函数
void interrupt_z_change() {
  counter = 0; 
}

void forceMeasure(){
  static boolean newDataReady = false;
  // check for new data/start next conversion:
  if (LoadCell_1.update()) newDataReady = true;
  LoadCell_2.update();

  //get smoothed value from data set
  if ((newDataReady)) {
    if (millis() > t_LC + serialPrintInterval) {
      float a = LoadCell_1.getData();
      float b = LoadCell_2.getData();
      Serial.print("F");
      Serial.print(timer_LC);
      Serial.print('\t');
      Serial.print(a);
      Serial.print('\t');
      Serial.println(b);
     
      newDataReady = 0;
      t_LC = millis();
      timer_LC = timer_LC + 0.050;
    }
  }
}

//
void runUsercmd(){
  switch(cmd){
    case 'S':
      temp = 0;
      counter = 0;
      timer_IE = 0;
      t_IE = 0; 
      onoff_angle = false;
      angleset = cmdint;
      Serial.print("C");
      Serial.print("Peel Angle has been set to ");
      Serial.print(cmdint);
      Serial.print(" deg.");
      Serial.println("  Angle Measurement is Prepared.");
      break;

    case 'R':
      temp = 0;
      counter = 0;
      timer_IE = 0;
      t_IE = 0; 
      onoff_angle = false;
      //angleset = 0;
      Serial.print("C");
      Serial.println("New Angle Measurement Ready.");
      break;
     
    case 'I':
      onoff_angle = true;
      Serial.print("C");
      Serial.println("Start Angle Measurement...");
      break;

    case 'i':
      onoff_angle = false;
      Serial.print("C");
      Serial.println("Angle Measurement Stopped.");
      break;  

    case 'L':
      onoff_force = true;
      Serial.print("c");
      Serial.println("Start Force Measurement...");
      break;

    case 'l':
      onoff_force = false;
      Serial.print("c");
      Serial.println("Force Measurement Stopped.");
      break;

    case 'r':
      onoff_force = false;
      t_LC = 0;
      timer_LC = 0;
      Serial.println("O");
      break;

    case 'B':
      onoff_test = true;
      
      break;

    case 'b':
      onoff_test = false;
      break;

    case 'E':
      onoff_test = false;
      onoff_force = false;
      onoff_angle = false;
      break;

    case 'N':
      temp = 0;
      counter = 0;
      timer_IE = 0;
      t_IE = 0; 
      onoff_angle = false;
      onoff_force = false;
      t_LC = 0;
      timer_LC = 0;

      Serial.println("n");
      break;
  }
}
