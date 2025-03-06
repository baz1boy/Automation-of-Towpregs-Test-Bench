#include <AccelStepper.h>
#include <Wire.h>
#include <math.h>
#include "WaveShare_MLX90615.h"

#define slaveAddress 0x08

WaveShare_MLX90615   MLX90615 = WaveShare_MLX90615();

// AccelStepper Setup
// index 1: use Motor driver to control microstep
AccelStepper stepper1(1, 2, 3);  const int ENA1 = 4;  //PUL- PIN2; DIR- PIN3; ENA- PIN4;
AccelStepper stepper2(1, 5, 6);  const int ENA2 = 7;  const int Motor12 = 40;
AccelStepper stepperDU(1, 8, 9); const int ENADU = 10;  const int MotorDU = 41;

//Mosfet Control
const int Peltier_Mosfet = 12;      //The N-Channel MOSFET is on digital pin 12
int power = 0; //Power level fro 0 to 100%
int peltier_level = map(power, 0, 100, 0, 255); //This is a value from 0 to 255 that actually controls the MOSFET


// User Command
char cmd;
String cmdString;
String cmdnum;
int cmdint;
String Arr[6];
int stringStart = 0;
int arrayIndex = 0;

//---------------Timer Setup
unsigned long TimeNow = 0;
unsigned long timerDU = 0;
unsigned long timerConti = 0;
bool count;
int reset;

//--------------Test Parameters
float tape_d;
float pangle;
float plength;
float pwidth;
float tape_T;

//------------Machine Parameters
const float drumD1 = 150;   // mm
const float drumD2 = 150;   // mm
const float D_gear_above = 60;    // mm
const float D_gear_below = 22;    // mm
const float i_gear = D_gear_above / D_gear_below;
const float distance = 281.5;   // mm
const int i_gearbox = 10;

//----------------Motor Parameters
const float stepdegree = 0.9;  // [°]
float MaxiSpeed = 2000;
float peelspeed = 1000;         // mm/min (Initial)
float drum1speed = 900;
float drum2speed = 1000;

float v_D2 = peelspeed;                          //drum2 velocity v  [mm/min]
float w_D2 = v_D2 * 2 / drumD2;                  //drum2 angular velocity w   [rad/min]
float w_gear2_above = w_D2;                      //gear2 above angular velocity w  [rad/min]
float w_gear2_below = i_gear * w_gear2_above;    //gear2 below angular velocity w  [rad/min]
float w_motor2 = i_gearbox * w_gear2_below / 60;      //motor2 angular velocity w  [rad/s]
float n_motor2 = (60 / 2 / PI) * w_motor2;       //motor2 rpm  [1/min]
float degreePerSec_motor2 = n_motor2 * 360 / 60;   // [°/s]

float motor2speed = degreePerSec_motor2 / stepdegree;  // motor2 speed in [steps/sec]
float speed_conversion_factor = 2 / drumD1 * i_gear * i_gearbox / 60 * 60 / 2 / PI * 360 / 60 / stepdegree; 
float motor1speed = drum1speed * speed_conversion_factor;

//----------------Drum Motors 
bool emergency = false;
bool DMready = false;
bool Manual;
bool DMrun = false;
unsigned long t_SpeedMeasure = 0;
const double serialPrintInterval = 50;  // 50ms


bool DMclick = false;
float DMclickStep = 100;
bool DMhold = false;
float DMholdSpeed = 400;
int move1or2 = 0;
int movedir;
bool DMtest = false;
bool testD1 = false;
bool testD2 = false;

//--------------Delivery Motor Control 
bool DUready = false;
float DUSpeed_a = 800;
float DUSpeed_m = 800;
bool DUrun = false;
bool DUtest = false;
bool DUhold = false;
float DUholdSpeed = 200;

//--------------I2C Send data
bool flag = false;
bool speedmeasure;
char I2C[16];
float angleChange;
float loadcell1;
float loadcell2;

//------------PID Control Motor
float setAngle;
float error = 0;
float pre_e = 0;
float pre_i = 0;
float kp = 1;
float ki = 0;
float kd = 0;
float dt = 0.050;  //50ms
float P, I, D, PID;

//-----------------TemperatureSensor
bool TemSensor = false;
unsigned long TemStart = 0;
unsigned long Tem_Interval = 0;

//-----------------Peltier Control
const int peltier1 = 22;      // 
const int peltier2 = 23;
const int peltier3 = 24;
const int peltier4 = 25;
const int fan = 26;
const int pump = 27;
const int PWM_fan = 11;       //if needed, control fan speed 500-2000rpm

//------------------PID Control Peltier
bool heating;
bool cooling;
bool water_cooler;
float setTemperature;
float temperature_read = 0.0;
float error_T = 0;
float pre_e_T = 0;
float pre_i_T = 0;
float kp_T = 9.1;
float ki_T = 0.3;
float kd_T = 1.8;
float dt_T = 0.100;
float P_T, I_T, D_T, PID_T;

//**************************************************************************************************************
//************************************************** Setup ****************************************************

void setup()
{
  pinMode(Motor12, OUTPUT);
  pinMode(MotorDU, OUTPUT);
  pinMode(peltier1, OUTPUT);
  pinMode(peltier2, OUTPUT);
  pinMode(peltier3, OUTPUT);
  pinMode(peltier4, OUTPUT);
  pinMode(Peltier_Mosfet, OUTPUT);
  pinMode(PWM_fan, OUTPUT);
  pinMode(fan, OUTPUT);
  pinMode(pump, OUTPUT);

  //digitalWrite(ENA1, HIGH);
  //digitalWrite(ENA2, HIGH);
  //digitalWrite(ENADU, HIGH);

  stepper1.setMaxSpeed(MaxiSpeed);
  stepper1.setSpeed(0);          
  
  stepper2.setMaxSpeed(MaxiSpeed);
  stepper2.setSpeed(0);          

  stepperDU.setMaxSpeed(MaxiSpeed);
  stepperDU.setSpeed(0);          

  // initialize MLX90615 Sensor 
  MLX90615.begin();  

  // initialize the serial port:
  Serial.begin(9600);
  
  // initialize I2C UNO and MEGA
  Wire.begin(slaveAddress);                       // adress as 'slaveAddress';
  Wire.onReceive(receiveEvent);
}

//*************************************************************************************************************
//**************************************************** Loop *************************************************
void loop() 
{ 
  if (Serial.available()) {     // 
    //cmd = Serial.read();        //   
    //cmdint = Serial.parseInt();
    cmdString = Serial.readString();
    cmd = cmdString.charAt(0);
    cmdnum = cmdString.substring(1);
    cmdint = cmdnum.toInt();

    runUsercmd();  
  }

  // I2C data transmission
  if(flag){
    speedControl();
    flag = false;
  }

  // Drum Motors
  if(DMready){
    if(!Manual){
      autoDM();    
    }else if(Manual){
      manaulDM();
    }

    if(speedmeasure){
      speedMeasure();
    }
  } 

  // Delivery Motor
  if(DUready){
    if(DUrun){
      runDU();
    }else if(DUtest){
      testDU();
    }else if(DUhold){
      holdDU();
    }
  }

//------------------------------ Temperature Control -------------------------------------
  if(TemSensor){
      temperatureSensor();
  }

  if(heating && !cooling){
    temperature_heating();
  }else if(cooling && !heating){
    temperature_cooling;
  }
}

//**********************************************************************************************************
//*************************************************** Functions *****************************************************

//------------------------------------------------------------------------------------------------------
//-------------------------------------------------- I2C recevied Data ----------------------------------

void receiveEvent(int howMany){
  for(int i = 0; i < howMany; i++){
    I2C[i] = Wire.read();
  }
  String readData = String(I2C);
  String I2C_array[3];
  int I2C_stringStart = 0;
  int I2C_arrayIndex = 0;
  for(int i = 0; i < readData.length(); i++){
    if(readData.charAt(i) == ','){
      I2C_array[I2C_arrayIndex] = "";
      I2C_array[I2C_arrayIndex] = readData.substring(I2C_stringStart, i);
      I2C_stringStart = i + 1;
      I2C_arrayIndex++;
    }
  }
  angleChange = I2C_array[0].toFloat();
  loadcell1 = I2C_array[1].toFloat();
  loadcell2 = I2C_array[2].toFloat();
  Serial.print(angleChange);
  Serial.print('\t');
  Serial.print(loadcell1);
  Serial.print('\t');
  Serial.println(loadcell2);
  flag = true;
}

//-----------------------------------------------------------------------------------------------------------
//------------------------------------------------- Temperature Sensor -------------------------------------------

void temperatureSensor(){
  float at = MLX90615.readAmbientTemp();    // 
  float ot = MLX90615.readObjectTemp(); //
  Tem_Interval = millis() - TemStart;
  
  if(Tem_Interval > 100)
  {
    Serial.print("T"); 
    Serial.print(at); 
    Serial.print('\t'); 
    Serial.println(ot); 

    TemStart = millis();
    Tem_Interval = 0;
  }

  temperature_read = ot;
}

//-----------------------------------------------------------------------------------------------------------
//------------------------------------------------- Temperature Controller -------------------------------------------

void temperature_heating(){
  error_T = setTemperature - temperature_read;   // define error 
  P_T = kp_T * error_T;                          // P output
  pre_i_T = pre_i_T + error_T * dt_T;            // accumulated error over time, with dt = 0.100s (100ms)
  I_T = ki_T * pre_i_T;                          // I output
  D_T = kd_T * ((error_T - pre_e_T) / dt_T);     // D output  
  PID_T = P_T + I_T + D_T;                       // PID output    
  pre_e_T = error_T;                             // save the previous error

  if(PID_T < 0)                                  // define PID output value in a PWM range between 0 and 255
  {    PID_T = 0;    }
  if(PID_T > 255)  
  {    PID_T = 255;  }

  analogWrite(PWM_fan,PID_T);                    // control the current through the Peltier with PWM signal
}

void temperature_cooling(){
  error_T = temperature_read - setTemperature;   // define error as difference between setpoint and measured value
  P_T = kp_T * error_T;                          // P output
  pre_i_T = pre_i_T + error_T * dt_T;            // accumulated error over time, with dt = 0.100s (100ms)
  I_T = ki_T * pre_i_T;                          // I output
  D_T = kd_T * ((error_T - pre_e_T) / dt_T);     // D output  
  PID_T = P_T + I_T + D_T;                       // PID output    
  pre_e_T = error_T;                             // save the previous error

  if(PID_T < 0)                                  // define PID output value in a PWM range between 0 and 255
  {    PID_T = 0;    }
  if(PID_T > 255)  
  {    PID_T = 255;  }

  analogWrite(PWM_fan,PID_T);                    // control the current through the Peltier with PWM signal
}

//------------------------------------------------------------------------------------------------------------------
//-------------------------------------------- Speed Control --------------------------------------------------

void speedControl(){        // 100mm/min  38.58steps/sec
  error = -angleChange;
  P = kp * error;
  pre_i = pre_i + error * dt;
  I = ki * pre_i;
  D = kd * ((error - pre_e) / dt);
  PID = P + I + D;
  pre_e = error;

  /*Serial.print("Error: "); Serial.print(error);
  Serial.print("  P: "); Serial.print(P);
  Serial.print("  I: "); Serial.print(I);
  Serial.print("  D: "); Serial.print(D);
  Serial.print("  PID: "); Serial.print(PID);*/

  //Motor Speed
  if(PID > 0){
    motor1speed = motor1speed + 1;
    drum1speed = motor1speed / speed_conversion_factor;
  }else if(PID < 0){
    motor1speed = motor1speed - 1;
    drum1speed = motor1speed / speed_conversion_factor;
  }

  if(motor1speed > motor2speed){
    motor1speed = motor2speed;
  }
  //Serial.print("  Drum1Speed: "); Serial.println(drum1speed);
}

void speedMeasure(){
  if(millis() > t_SpeedMeasure + serialPrintInterval){
    Serial.print("J");
    Serial.print(drum1speed);
    //Serial.print('\t');
    //Serial.print(motor1speed);
    Serial.print('\t');
    Serial.println(drum2speed);
    //Serial.print('\t');
    //Serial.println(motor2speed);
        
    t_SpeedMeasure = millis();
  }    
}

//----------------------------------------------------------------------------------------------------------------
//-------------------------------------------- Drum Motors Auto Run -------------------------------------------
void autoDM(){
  if(!emergency){
    if(DMrun){
      stepper1.setMaxSpeed(MaxiSpeed);
      stepper2.setMaxSpeed(MaxiSpeed);
      stepper1.setSpeed(motor1speed);
      stepper2.setSpeed(motor2speed);
  
      stepper1.runSpeed();
      stepper2.runSpeed();
     }
     else{
      stepper1.setSpeed(0);
      stepper2.setSpeed(0);
     } 
  }else{
    DMready = false;
    DMrun = false;
    speedmeasure = false;
    stepper1.setSpeed(0);
    stepper2.setSpeed(0);
  }
}

//------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------- Drum Motors Manual -------------------------------------------
void manaulDM(){
  if(DMclick && !DMhold){
    stepper1.setMaxSpeed(DMclickStep);    // 
    stepper1.setAcceleration(50.0);  // 

    stepper2.setMaxSpeed(DMclickStep);    
    stepper2.setAcceleration(50.0);  
    
    stepper1.run();
    stepper2.run();
  }

  if(DMhold && !DMclick)
  {
    if(move1or2 == 2){ 
      stepper1.setSpeed(0); 
      if(movedir == 0){
        stepper2.setMaxSpeed(MaxiSpeed);
        stepper2.setSpeed(DMholdSpeed); 
      }else if(movedir == 1){
        stepper2.setMaxSpeed(MaxiSpeed);
        stepper2.setSpeed(-DMholdSpeed);
      }else if(movedir == 3){
        stepper2.setSpeed(0);
        move1or2 = 0;
      }
      stepper2.runSpeed();
    }
    else if(move1or2 == 1)
    {
      stepper2.setSpeed(0); 
      if(movedir == 0){
        stepper1.setMaxSpeed(MaxiSpeed);
        stepper1.setSpeed(DMholdSpeed);
      }else if(movedir == 1){
        stepper1.setMaxSpeed(MaxiSpeed);
        stepper1.setSpeed(-DMholdSpeed);
      }else if(movedir == 3){
        stepper1.setSpeed(0);
        move1or2 = 0;
      }
      stepper1.runSpeed();
    }
  }

  if(DMtest){
    if(!testD1 && !testD2){
      DMtest = false;
    }else if(testD1 && !testD2){
      stepper1.setMaxSpeed(MaxiSpeed);
      stepper1.setSpeed(motor1speed);
      stepper2.setMaxSpeed(MaxiSpeed);
      stepper2.setSpeed(0);    
      //Serial.println(motor1speed);
    }else if(testD2 && !testD1){
      stepper1.setMaxSpeed(MaxiSpeed);
      stepper1.setSpeed(0);
      stepper2.setMaxSpeed(MaxiSpeed);
      stepper2.setSpeed(motor2speed);   
      //Serial.println("motor2speed");
    }else if(testD1 && testD2){
      stepper1.setMaxSpeed(MaxiSpeed);
      stepper1.setSpeed(motor1speed);
      stepper2.setMaxSpeed(MaxiSpeed);
      stepper2.setSpeed(motor2speed);   
      //Serial.println("motor2speed");
    }
    stepper1.runSpeed(); 
    stepper2.runSpeed();
  }
}

//-----------------------------------------------------------------------------------------------------------------
//----------------------------------------------------- Delivery Motor ---------------------------------------
void runDU(){
  timerDU = millis() - TimeNow;
  if(timerDU >= 30000){
    DUSpeed_a = -DUSpeed_a;
    TimeNow = millis();
    timerDU = 0;
  }
  stepperDU.setSpeed(DUSpeed_a);
  stepperDU.runSpeed();
}

void testDU(){ 
  stepperDU.setSpeed(DUSpeed_m);
  stepperDU.runSpeed();
}

void holdDU(){
  stepperDU.setSpeed(DUholdSpeed);
  stepperDU.runSpeed();
}

//**************************************************************************************************
//********************************************** User cmd ******************************************
void runUsercmd(){
  switch(cmd){
    case 'A':
      for(int i = 0; i < cmdnum.length(); i++){
        if(cmdnum.charAt(i) == ','){
          Arr[arrayIndex] = "";
          Arr[arrayIndex] = cmdnum.substring(stringStart, i);

          stringStart = i + 1;
          arrayIndex++;
        }
      }
      tape_d = Arr[0].toFloat();
      pangle = Arr[1].toFloat();
      peelspeed = Arr[2].toFloat();
      plength = Arr[3].toFloat();
      pwidth = Arr[4].toFloat();
      tape_T = Arr[5].toFloat();

      setTemperature = tape_T;
      drum2speed = peelspeed;
      drum1speed = drum2speed - 50;
      motor2speed = drum2speed * speed_conversion_factor;
      motor1speed = drum1speed * speed_conversion_factor;
      if(cmd != 'a')
      {
      Serial.println("a");      // send a feedback that Arduino variables updated
      }
      
      /*Serial.println(tape_d);
      Serial.println(pangle);
      Serial.println(peelspeed);
      Serial.println(plength);
      Serial.println(pwidth);
      Serial.println(tape_T);  */
      
      stringStart = 0;
      arrayIndex = 0;
      break; 
    
          
    case 'N':
      emergency = false;
      DMrun = false;
      DMready = true;
      stepper1.setSpeed(0);
      stepper2.setSpeed(0);
      peelspeed = 1000;
      drum2speed = 1000;
      drum1speed = 900;
      motor2speed = drum2speed * speed_conversion_factor;
      motor1speed = drum1speed * speed_conversion_factor;
      Serial.println("Q");
      break;
         
//------------------------------------------- Drum Motors Control -----------------------------------------------

    case 'C':
      DMready = true;
      digitalWrite(Motor12, HIGH);
      break;

    case 'c':
      DMrun = false;
      DMready = false;
      speedmeasure = false;
      digitalWrite(Motor12, LOW);
      break;

    case 'M':
      Manual = true;
      DMrun = 0;
      break;

    case 'm':
      Manual = false;
      DMrun = 0;
      break;

    case 'X':
      emergency = true;
      break;

    case 'G':
      DMrun = 1;  
      Serial.println("P");
      speedmeasure = true;
      break;

    case 'g':
      DMrun = 0;
      speedmeasure = false;
      break;

    //------------------------- Motor 1 moves by one click
    case 'Y':
      DMhold = false;
      DMclick = true;
      if(cmdint == 0){
        stepper1.move(DMclickStep);
      }else{
        stepper1.move(-DMclickStep);
      }
      stepper2.move(0);
      break;

    //------------------------  Motor 2 moves by one click
    case 'y':
      DMhold = false;
      DMclick = true;
      if(cmdint == 0){
          stepper2.move(DMclickStep);
      }else{
          stepper2.move(-DMclickStep);
      }  
      stepper1.move(0); 
      break;

    //-------------------------   Motor 1 hold to move
    case 'L':
      DMclick = false;
      DMhold = true;
      move1or2 = 1;
      movedir = cmdint;
      break;

    //---------------------------  Motor 2 hold to move
    case 'l':
      DMclick = false;
      DMhold = true;
      move1or2 = 2;
      movedir = cmdint;
      break;

    // Set Drum Speed
    case 'V':
      if(cmdnum.charAt(0) == 'L'){
        drum1speed = cmdnum.substring(1).toInt();
        motor1speed = drum1speed * speed_conversion_factor;
      }else if(cmdnum.charAt(0) == 'R'){
        drum2speed = cmdnum.substring(1).toInt();
        motor2speed = drum2speed * speed_conversion_factor;
        //Serial.println(motor2speed);
      }else if(cmdnum.charAt(0) == 'P'){
        drum2speed = cmdnum.substring(1).toInt();
        drum1speed = drum2speed - 50;
        peelspeed = drum2speed;
        motor2speed = drum2speed * speed_conversion_factor;
        motor1speed = drum1speed * speed_conversion_factor;
        //Serial.println(motor2speed);
      }

    case 'x':
      DMtest = true;
      if(cmdint == 1){
        testD1 = true;
      }
      if(cmdint == 2){
        testD2 = true;
      }
      if(cmdint == 0){
        testD1 = false;
      }
      if(cmdint == 3){
        testD2 = false;
      } 
      break;

 
//------------------------------------------ Delivery Motor Control ----------------------------------------
    
    // Motor Connection
    case 'O':
      DUready = true;
      digitalWrite(MotorDU, HIGH);
      break;

    case 'o':
      stepperDU.setSpeed(0);
      DUrun = false; 
      DUready = false;
      DUSpeed_a = 800;    // Inital Speed
      DUSpeed_m = 800;
      digitalWrite(MotorDU, LOW);
      break;

    // DU Auto loop
    case 'D':
      DUrun = true; 
      Serial.println("p");
      TimeNow = millis() - timerConti;
      break;

    case 'd':
      DUrun = false; 
      stepperDU.setSpeed(0);
      timerConti = timerDU;
      break;
      
    // Reset
    case 'n':
      TimeNow = 0;
      timerDU = 0;
      timerConti = 0;
      DUSpeed_a = abs(DUSpeed_a);
      Serial.println("q");
      break;  
      
    // Manual test
    case 'K':
      DUtest = true;
      if (cmdint == 0){
        DUSpeed_m = abs(DUSpeed_m);
      }
      else if(cmdint == 1){
        DUSpeed_m = -abs(DUSpeed_m);
      }
      break;

    case 'k':
      DUtest = false;
      DUSpeed_m = abs(DUSpeed_m);
      stepperDU.setSpeed(0);
      break;

    // Hold to Move
    case 'H':
      DUhold = true;
      if (cmdint == 0){
        DUholdSpeed = abs(DUholdSpeed);
      }
      else if(cmdint == 1){
        DUholdSpeed = -abs(DUholdSpeed);
      }
      break;

    case 'h':
      DUhold = false;
      stepperDU.setSpeed(0);
      DUholdSpeed = 100;
      break;

    // Set Speed
    case 'v':
      if(cmdnum.charAt(0) == 'a'){
        DUSpeed_a = cmdnum.substring(1).toInt();
        Serial.print("j");
        Serial.println("Auto Speed Setting Completed.");
      }else if(cmdnum.charAt(0) == 'm'){
        DUSpeed_m = cmdnum.substring(1).toInt();
        Serial.print("j");
        Serial.println("Manual Speed Setting Completed.");
      }
      break;
      
//------------------------------------- Temperature Sensor ------------------------------------------  
    
    case 'W':
      TemSensor = true;
      TemStart = millis();
      break;

    case 'w':
      TemSensor = false;
      Tem_Interval = 0;
      TemStart = 0;
      break;

//------------------------------------------- Peltier -------------------------------------------

    case 'J': 
      digitalWrite(peltier1, HIGH);   //Heating
      digitalWrite(peltier2, LOW);
      digitalWrite(peltier3, LOW);
      digitalWrite(peltier4, HIGH);
      heating = true;
      cooling = false;
      error_T = 0;
      pre_e_T = 0;
      pre_i_T = 0; 
      break;

    case 'j':
      digitalWrite(peltier1, LOW);   //Cooling
      digitalWrite(peltier2, HIGH);
      digitalWrite(peltier3, HIGH);
      digitalWrite(peltier4, LOW);
      digitalWrite(fan, HIGH);
      digitalWrite(pump, HIGH);
      heating = false;
      cooling = true;
      water_cooler = true;
      error_T = 0;
      pre_e_T = 0;
      pre_i_T = 0; 
      break;

    case 'B':
      digitalWrite(peltier1, LOW);
      digitalWrite(peltier2, LOW);
      digitalWrite(peltier3, LOW);
      digitalWrite(peltier4, LOW);
      heating = false;
      cooling = false;
      error_T = 0;
      pre_e_T = 0;
      pre_i_T = 0; 
      break;

    case 'b':
      if(water_cooler){
        digitalWrite(fan, LOW);
        digitalWrite(pump, LOW);
        water_cooler = false;
      }else{
        digitalWrite(fan, HIGH);
        digitalWrite(pump, HIGH);
        water_cooler = true;
      }
      break;
      
//----------------------------------------- Close Port ----------------------------------------------
    
    case 'E':
      DMrun = false;
      DUready = false;
      DUrun = false;
      DMready = false;
      stepper1.setSpeed(0);
      stepper2.setSpeed(0);
      stepperDU.setSpeed(0);
      digitalWrite(Motor12, LOW);
      digitalWrite(MotorDU, LOW);
      
      TemSensor = false;
      break;
  }
}
