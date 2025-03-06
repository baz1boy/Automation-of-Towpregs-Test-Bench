#include <AccelStepper.h>
#include <Wire.h>
#include <math.h>

#define slaveAddress 0x08

AccelStepper stepper1(1, 2, 3);  int ENA1 = 4;   //PUL- PIN2; DIR- PIN3; ENA- PIN4;
AccelStepper stepper2(1, 5, 6);  int ENA2 = 7;

float MaxiSpeed = 2000;

// User Command
char cmd;
String cmdString;
String cmdnum;
int cmdint;

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
bool DMready = false;
bool DMrun = false;
unsigned long t_SpeedMeasure = 0;
const double serialPrintInterval = 50;  // 50ms


//--------------I2C Send data
bool flag = false;
bool speedmeasure;
char I2C[16];
float angleChange;


//------------PID Control
float setAngle;
float error = 0;
float pre_e = 0;
float pre_i = 0;
float kp = 0;
float ki = 0;
float kd = 0;
float dt = 0.050;  //50ms
float P, I, D, PID;


void setup() {
  pinMode(ENA1, OUTPUT);
  pinMode(ENA2, OUTPUT);

  digitalWrite(ENA1, HIGH);
  digitalWrite(ENA2, HIGH);

  stepper1.setMaxSpeed(MaxiSpeed);
  stepper1.setSpeed(0);          // Set Motor 1 Speed at  steps/sec
  
  stepper2.setMaxSpeed(MaxiSpeed);
  stepper2.setSpeed(0);          // Set Motor 2 Speed at  steps/sec

  Serial.begin(9600);

  Wire.begin(slaveAddress);                       // adress as '2';
  Wire.onReceive(receiveEvent);
}

void loop() {
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

  if(DMready){
    autoDM();    
  }
}


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

  //Serial.println(angleChange);
  flag = true;
}

void speedControl(){        
  error = -angleChange;                   // define error as the angle deviation
  P = kp * error;                         // P output
  pre_i = pre_i + error * dt;             // accumulated error over time, with dt = 0.05s (50ms)
  I = ki * pre_i;                         // I output
  D = kd * ((error - pre_e) / dt);        // D output
  PID = P + I + D;                        // PID output
  pre_e = error;                          // save the previous error

  Serial.print("Error: "); Serial.print(error);
  Serial.print("  P: "); Serial.print(P);
  Serial.print("  I: "); Serial.print(I);
  Serial.print("  D: "); Serial.print(D);
  Serial.print("  PID: "); Serial.print(PID);

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
  Serial.print("  Drum1Speed: "); Serial.println(drum1speed);
}

void autoDM(){
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
}

void runUsercmd(){
  switch(cmd){
    case 'A':
      DMready = true; 
      break;

    case 'a':
      DMready = false;
      break;

    case 'B':
      DMrun = true;
      break;

    case 'b':
      DMrun = false;

    case 'V':
      drum2speed = cmdint;
      drum1speed = drum2speed - 50;
      motor2speed = drum2speed * speed_conversion_factor;
      motor1speed = drum1speed * speed_conversion_factor;
      Serial.println(drum2speed);
      break;
  
    case 'E':
      DMrun = false;
      DMready = false;
      stepper1.setSpeed(0);
      stepper2.setSpeed(0);
      drum2speed = 1000;
      drum1speed = 900;      
      motor2speed = drum2speed * speed_conversion_factor;
      motor1speed = drum1speed * speed_conversion_factor;
      break;
  }

}
