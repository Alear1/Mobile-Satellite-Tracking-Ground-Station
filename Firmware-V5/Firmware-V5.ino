/*
Ground station ground station pointing system
controller firmware written for Carleton University Designated Study.
Author: August A. Lear
Student #: 101075941
Current Version Date: 2020-09-22
This version not for public distribution

Use following commands to test(Tested on Ubuntu version 20):
~$ sudo rotctld -m 202-- -r /dev/ttyACM0 -s 115200
If the above fails, tr:
~$ sudo chmod a+rw /dev/ttyACM0
Then try rotctld command line again. Make sure arduino Serial Monitor is CLOSED
In a second terminal:
~$ gpredict
-navigate to rotator control window in gpredict GUI
-engage rotator


Keywords:
FIXME   something that needs to be fixed/updated for new hardware
ASSUMING  states an assumption that has been made

other notes:
Future version should have an encoder/magnetometer azimuth sensor
capabilities.
*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <LiquidCrystal.h>
#include <AccelStepper.h> //INSTALL WITH LIBRARY MANAGER

//SENSOR PIN DEFINITION
#define AZPOT A0 //Pin set for azimuth potentiometer
#define ELPOT A1//Pin set for elevation potentiometer
#define EL_LOWSTOP 12 //elevation low-point endstop switch
#define EL_HIGHSTOP 13 //elevation high-point endstop switch
#define AZ_HOMESWITCH 22 //Homing point switch for azimuth

//MOTOR PIN DEFINITIONS
#define STEP_PIN_AZ 8//Pin set for stepper motor 1 step pin
#define DIR_PIN_AZ 9//pin set for stepper motor 1 direction pin
#define EN_PIN_AZ 14//Pin for stepper motor 1 enable
#define STEP_PIN_EL 10 //Pin set for stepper motor 2 step pin
#define DIR_PIN_EL 11 //Pin set for stepper motor 2 direction pin
#define EN_PIN_EL 15 //Pin set for stepper motor 2 enable

//DATA CONSTANT SETUP
#define BAUDRATE 115200 //BAUD rate of serial connection
#define BUFFERLEN 64  //Length of data array sent and received
#define CYCLEWAIT 100//delay at end of main loop (millis)

//CALIBRATION CONSTANT SETUP
#define EL_CAL_TIMEOUT 60000 //milliseconds until elevation calibration timeout
#define AZ_CAL_TIMEOUT 120000 //milliseconds until azimuth calibration timeout
#define EL_CAL_RATE 5 //angular rate(deg/s) of calibration for elevation
#define AZ_CAL_RATE 5 //angular rate(deg/s) of calibration for azimuth

//HARDWARE ABSTRACTION LAYER(for now...)
#define EL_DEG_PER_STEP 0.018 //Number of degrees per step(approximate) of the elevation system
#define AZ_DEG_PER_STEP 0.018 //Num of degrees per step (approx) of the azimuth system
#define STEP_LIMIT 1000 //steps/second limit

//Define pins for the LCD
const int rs = 3, e = 2, d4 = 4, d5 = 5, d6 = 6, d7 = 7;  //FIXME (replace with I2C enabled LCD)
LiquidCrystal lcd(rs, e, d4, d5, d6, d7); //instatiate lcd object

//Instantiate Azimuth and Elevation stepper objects
AccelStepper AzStepper(AccelStepper::DRIVER, STEP_PIN_AZ, DIR_PIN_AZ);
AccelStepper ElStepper(AccelStepper::DRIVER, STEP_PIN_EL, DIR_PIN_EL);

//MISC DEFINITIONS
bool err = 0; //tracks errors
bool Debug = 0; //MAKE 1 FOR DEBUG, 0 FOR NORMAL(DEV)

int chCount = 0;  //data reception counter
int azInt = 0, azFlt = 0, elInt = 0, elFlt = 0; //For sprintf
int n = 0; //counter for azimuth crossover direction
float azPos;  //Azimuth position according to sensors(deg)
float elPos;  //Elevation position according to sensors(deg)
float azTgt = 0;  //Azimuth target coordinate(deg)
float elTgt = 0;  //Elevation target coordinate(deg)
float Tolerance = 1;//raw tolerance for difference between tgt and pos(deg)

char inChar[BUFFERLEN]; //Array into which data is received
char outChar[BUFFERLEN];  //Array to be transmitted

int elMaxPoint; //elevation sensor reading at elevation high limit switch
int elMinPoint; //elevation sensor reading at elevation low limit switch

void setup() {
  //Begin serial communications
  Serial.begin(BAUDRATE);
  //Begin communication to 16-position, 2 line lcd
  lcd.begin(16,2);
  lcd.noAutoscroll(); //prevents LCD from autoscrolling
  
  //setup homing switch pins
  pinMode(EL_LOWSTOP, INPUT);
  pinMode(EL_HIGHSTOP, INPUT);
  pinMode(AZ_HOMESWITCH, INPUT);
  sensorPos(); //get initial sensor position
  Serial.println("Startup");

  //setting max step rate of motors:
  AzStepper.setMaxSpeed(STEP_LIMIT);
  ElStepper.setMaxSpeed(STEP_LIMIT);

  //Enabling Motors:
  digitalWrite(EN_PIN_AZ, HIGH);
  digitalWrite(EN_PIN_EL, HIGH);
  
  
  //calibrate();//goes to calibrate function(comment to disable)
}

void loop() {
  //Read new byte into Data, then add that
  //byte to the inChar array
  while(Serial.available() > 0) {
    inChar[chCount] = Serial.read();
    chCount += 1;
  }
  delay(CYCLEWAIT/2); //Wait for all data  transfer operations to finish
  //If new data is received:
  
  if (chCount != 0 && Serial.available() < 1){
    if(inChar[0] == 'A' && inChar[2] != ' '){
      setTgt();
    }else if(inChar[0] == 'S'){
      Serial.println("Stop Command"); //Flush block called
    }else{
      //Break positions into parts so the sprintf() function can work:
      azInt = int(azPos);                 //Whole part of the azPos float
      azFlt = int((azPos - azInt) * 10);  //Decimal part of azPos float
      elInt = int(elPos);                 //Whole part of the elPos float
      elFlt = int((elPos - elInt) * 10);  //Decimal part of the elPos float

      sprintf(outChar, "AZ%d.%d EL%d.%d, azInt, azFlt, elInt, elFlt");
      Serial.write(outChar);
      Serial.flush(); //holds program while Serial.write sends data
    }
    memset(inChar, 0, sizeof inChar); //Reset inChar array
  }
  
  chCount = 0;  //reset counter
  lcdUpdate();  //update LCD
  motorUpdate();  //update motors
  delay(CYCLEWAIT/2); //wait
}

void lcdUpdate(){
  //Updates lcd with values of target and position
  char lcdOut1[16];
  char lcdOut2[16];
  
  sprintf(lcdOut1, "PA:%d|PE:%d", int(azPos), int(elPos)); //FIXME? 
  sprintf(lcdOut2, "TA:%d|TE:%d", int(azTgt), int(elTgt)); //FIXME ?

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(lcdOut1);//writing from top-left of lcd
  lcd.setCursor(0,1);//writing from bottom-left of LCD
  lcd.print(lcdOut2); 
}

void sensorPos(){
  //POTENTIOMETER MAPPING TO AZ AND EL POSITIONS IS TEMPORARY
  azPos = analogRead(AZPOT)/10 * 3.5 + n; //FIXME
  
  //Maps current position to between 0 and 90 degrees based on
  //limit switch position. map() is integer, so everything is kept at
  //100 times the required and then divided to form the float elPos
  //ASSUMES the maximum angle achievable is 90 degrees
  elPos = map(analogRead(ELPOT), elMinPoint, elMaxPoint, 0, 900)/100;
  
  //If azimuth crossover occurs:
  if(azPos >= 360){
    n -= 360;
  }else if(azPos < 0){
    n += 360;
  }

  if(Debug){
    Serial.println(" ");
    Serial.print("Az pos  El pos: ");
    Serial.print(azPos);
    Serial.print("  ");
    Serial.println(elPos);
  }
}

void setTgt(){
  //Sets azTgt and elTgt by parsing input string in inChar
  //start by getting azimuth target:
  char indFrom = 'Z'; //To search from this value
  char indTo = ' '; //Search up to this value
  int carry = 0;
  int valLen = 0;

  valLen = getLenBetween(inChar, indFrom, indTo);
  carry = getIndOf(inChar, indFrom);
  azTgt = excludingPlaces(inChar, carry+1, carry+valLen);

  indFrom = 'L';
  valLen = getLenBetween(inChar, indFrom, indTo);
  carry = getIndOf(inChar, indFrom);
  elTgt = excludingPlaces(inChar, carry+1, carry+valLen);
  
  if(Debug){
    Serial.println("Command Received ");
    Serial.println(valLen);
    Serial.print("AZ target is: ");
    Serial.println(azTgt);
    Serial.println(inChar);
  }
}

void motorUpdate(){
  //With this system the pointing is always one second behind required.
  sensorPos();//update sensors
  float dEl = elTgt - elPos;
  float azDist;

  
  azDist = abs(azTgt - azPos);
  if(getElSensors() == HIGH){ //check for el limit(homing) sensors
    azimuthRate(0);
    elevationRate(0);
  }else{
    //multiple cases for azimuth speed and direction:
    if(azDist > 180 && azPos > 180){
      azDist = (360 - azPos) + azTgt;
      azimuthRate(azDist);
    }else if(azDist >= 180 && azPos <= 180){
      azDist = azPos + (360 - azTgt);
      azimuthRate(-azDist);
    }else if(azDist < 180 && azPos < azTgt){
      azimuthRate(azDist);
    }else if(azDist > 180 && azPos > azTgt){
      azimuthRate(-azDist);
    }else{
      runError();
    }
    //setting elevation rate:
    elevationRate(dEl);
  }

  if(Debug){
    Serial.println("Azimuth rate is: ");
    //Serial.print(dAz);
    Serial.println("Elevation rate is: ");
    Serial.print(dEl);
  }
}

void calibrate(){
  //Currently doesn't actually calibrate anything, just goes to the homing
  //switches to make sure everything is working. timeouts exist.
  
  int t;//time at beginning of calibration action
  int dt;//time between beginning of calibration and current loop
  
  //Write information to LCD
  lcd.clear();
  lcd.print("Calibrating...");

  //first go to top elevation switch:
  elevationRate(EL_CAL_RATE);
  t = millis();
  while(digitalRead(EL_HIGHSTOP) != HIGH){
    dt = millis();
    if(dt - t > EL_CAL_TIMEOUT){calError();}
    if(digitalRead(EL_HIGHSTOP) == HIGH){calError();}
  }
  elevationRate(0);
  delay(1000);
  elMaxPoint = analogRead(ELPOT);
  
  //go to bottom elevation switch
  elevationRate(-EL_CAL_RATE);
  t = millis();
  while(digitalRead(EL_LOWSTOP) != HIGH){
    dt = millis();
    if(dt - t > EL_CAL_TIMEOUT){calError();}
    if(digitalRead(EL_HIGHSTOP) == HIGH){calError();}
  }
  elevationRate(0);
  delay(1000);
  elMinPoint = analogRead(ELPOT);
  //elevation now finished
  
  //Find azimuth homing switch:
  azimuthRate(AZ_CAL_RATE);
  t = millis();
  while(digitalRead(AZ_HOMESWITCH) != HIGH){
    AzStepper.runSpeed();
    dt = millis();
    if(dt - t < AZ_CAL_TIMEOUT){calError();}
  }
  azimuthRate(0);
  AzStepper.runSpeed();
  
  lcd.clear();
  lcd.print("Calibration Done");
  delay(1000);  //pause for effect
}

void elevationRate(float degRate){
  //argument is angular rate in deg/s (positive angle is positive rate)
  //makes elevation motor run at <argument> deg/s
  float stepRate;
  stepRate = degRate / EL_DEG_PER_STEP;
  if(stepRate > STEP_LIMIT){
    runError(); //FIXME same as azimuth rate fixme
  }else{
    ElStepper.setSpeed(stepRate);
    ElStepper.runSpeed(); 
  }
}

void azimuthRate(float degRate){
  //argument is angular rate in deg/s (clockwise is positive)
  //makes azimuth motor run at <argument> deg/s  
  float stepRate;
  stepRate = degRate / AZ_DEG_PER_STEP;
  if(abs(stepRate) > STEP_LIMIT){
    runError(); //FIXME  currently throws an error, could also default to
                       //just running at maximum value in either direction
  }else{
    AzStepper.setSpeed(stepRate);
    AzStepper.runSpeed();
  }
}

void calError(){
  //called when an error occurs in calibration
  //effectively pauses the system until it is restarted
  lcd.clear();
  lcd.print("CALIBRATION ERR");
  AzStepper.setSpeed(0);
  ElStepper.setSpeed(0);
  AzStepper.runSpeed();
  ElStepper.runSpeed();
  Serial.print("Calibration Error");
  while(HIGH){
    continue;//do nothing
  }
}

void runError(){
  //called when an error occurs in tracking
  //effectively pauses the system until it is restarted
  lcd.clear();
  lcd.print("RUNNING ERROR");
  AzStepper.setSpeed(0);
  ElStepper.setSpeed(0);
  AzStepper.runSpeed();
  ElStepper.runSpeed();
  Serial.print("RUN ERROR");
  while(HIGH){
    continue;//do nothing
  }
}

int getLenBetween(char valIn[], char srcFrom, char srcTo){
  //Returns number of spaces between first instance of initial character
  //and first instance of final character in remaining array
  int startCount = 0;
  int endCount = 64;
  char current;
  
  for(int i = 0; valIn[i] != srcFrom; i++){
     startCount = i+1;
  }
  for(int i = startCount; valIn[i] != srcTo; i++) {
     endCount = i;
  }
  return (endCount - startCount);
}

int getIndOf(const char str[], char src){
  //Returns index of character src in array str[]
  int out = 0;
  String newString = str;
  out = newString.indexOf(src);
  return out;
}

int getElSensors(){
  //returns 0 if no elevation limit switches are currently depressed,
  //returns 1 if ANY elevation sensor is currently depressed
  if(digitalRead(EL_LOWSTOP) == HIGH || digitalRead(EL_HIGHSTOP) == HIGH){
    return 1;
  }else{
    return 0;
  }
}

float excludingPlaces(const char str[64], int beg, int fin){
  //returns a float of only characters between indices beg, fin(inclusive)
  //from a string str. 64 char maximum.
  String newStr = str;
  float out;
  newStr.remove(0, beg);
  newStr.remove(fin);
  out = newStr.toFloat();
  return out;
}
