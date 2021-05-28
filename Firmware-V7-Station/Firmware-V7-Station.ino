/*
Ground station ground station pointing system
controller firmware written for Carleton University Designated Study.
Author: August A. Lear
Student #: 101075941
Current Version Date: 2021-02-05
This version not for public distribution

Windows Hamlib Command:
rotctl -m 202 -r COM4 -s 115200

Use following commands to test(Tested on Ubuntu version 20):
~$ sudo rotctld -m 202-- -r /dev/ttyACM0 -s 115200
If the above fails, tr:
~$ sudo chmod a+rw /dev/ttyACM0
Then try rotctld command line again. Make sure arduino Serial Monitor is CLOSED
In a second terminal:
~$ gpredict
-navigate to rotator control window in gpredict GUI
-engage rotator

Example test command:
AZ12.5 EL12.4

sets azimuth target to 12.5 degrees, sets EL target to 12.4 degrees

Keywords:
FIXME   something that needs to be fixed/updated for new hardware
ASSUMING  states an assumption that has been made

other notes:
Future version should have an encoder/magnetometer azimuth sensor
capabilities.

THIS CAN BE CONTROLLED WIRELESSLY OR WIRED
CONTROLLER SOFTWARE IS Station_Controller_V1
pins 48 CE and 49 CSN used for transciever connection
MEGA SPI PINS:
MOSI: 51
MISO: 50
SCK:  52
NRF24 pinout in this folder
NO LCD on this one, the LCD would be on the control side
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <AccelStepper.h> //INSTALL WITH LIBRARY MANAGER
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//USED PINS: A0,A1,8,9,14,25,26,27,29,35,37,39,48,49

//SENSOR PIN DEFINITION
#define AZPOT A0 //Pin set for azimuth potentiometer
#define ELPOT A1//Pin set for elevation potentiometer
#define EL_LOWSTOP 25 //elevation low-point endstop switch
#define EL_HIGHSTOP 27 //elevation high-point endstop switch
#define AZ_HOMESWITCH 29 //Homing point switch for azimuth

//MOTOR PIN DEFINITIONS
#define STEP_PIN_AZ 8//Pin set for stepper motor 1 step pin
#define DIR_PIN_AZ 9//pin set for stepper motor 1 direction pin
#define EN_PIN_AZ 14//Pin for stepper motor 1 enable
#define STEP_PIN_EL 37 //Pin set for stepper motor 2 step pin
#define DIR_PIN_EL 35 //Pin set for stepper motor 2 direction pin
#define EN_PIN_EL 39 //Pin set for stepper motor 2 enable

//DATA CONSTANT SETUP
#define BAUDRATE 115200 //BAUD rate of serial connection
#define BUFFERLEN 32 //Length of data array sent and received
#define CYCLEWAIT 100//delay at end of main loop (millis)

//CALIBRATION CONSTANT SETUP
#define EL_CAL_TIMEOUT 60000 //milliseconds until elevation calibration timeout
#define AZ_CAL_TIMEOUT 120000 //milliseconds until azimuth calibration timeout
#define EL_CAL_RATE 5 //angular rate(deg/s) of calibration for elevation
#define AZ_CAL_RATE 5 //angular rate(deg/s) of calibration for azimuth

//HARDWARE ABSTRACTION LAYER(for now...)
#define EL_DEG_PER_STEP 0.018 //Number of degrees per step(approximate) of the elevation system
#define AZ_DEG_PER_STEP 0.018 //Num of degrees per step (approx) of the azimuth system
#define STEP_LIMIT 800 //steps/second limit

//POLARITY SWITCHING
#define POLARITY_SWITCH_1 40
#define POLARITY_SWITCH_2 42

//Polarity Switching:
bool polarity_1 = 0;
bool polarity_2 = 0;

//Instantiate Azimuth and Elevation stepper objects
AccelStepper AzStepper(AccelStepper::DRIVER, STEP_PIN_AZ, DIR_PIN_AZ);
AccelStepper ElStepper(AccelStepper::DRIVER, STEP_PIN_EL, DIR_PIN_EL);

//Instantiate Wireless transciever:
RF24 radio(48,49); //CE, CSN

//MISC DEFINITIONS
bool err = 0; //tracks errors
bool Debug = 0; //MAKE 1 FOR DEBUG, 0 FOR NORMAL(DEV)

int chCount = 0;  //data reception counter
int azInt = 0, azFlt = 0, elInt = 0, elFlt = 0; //For sprintf
int n = 0; //counter for azimuth crossover direction
float Tolerance = 0.1;//raw tolerance for difference between tgt and pos(deg)
float azBias = 0; //constant azimuth bias
float elBias = 0; //constant elevation bias

//Radio:
bool wireSelect = 1; //0 for Wired, 1 for wireless
int skipLoops = 50; //number of loops to skip
int currentLoop = 0;
const byte addressA[6] = "00001";     //Address byte to write to
const byte addressB[6] = "00002";     //Address byte to read from

struct station_rx {
  //el_target contains the elevation target being sent from GPredict
  float el_target;
  //az_target contains the elevation target being sent from GPredict
  float az_target;
  //command contains a byte, which defines a special command.
  int command;
};
struct station_tx {
  //el_position contains the elevation position being received from the station
  float el_position;
  //az_position contains the azimuth position being received from the station
  float az_position;

  bool command_understood;
};

station_tx txData;
station_rx rxData;

char inChar[BUFFERLEN]; //Array into which data is received SERIAL
char outChar[BUFFERLEN];  //Array to be transmitted SERIAL

int elMaxPoint; //elevation sensor reading at elevation high limit switch
int elMinPoint; //elevation sensor reading at elevation low limit switch

//DEBUGGING
unsigned long timeFlag = 0;
unsigned long timeDif = 0;

void setup() {
  //The idea is to set up both the serial and wireless connections, so that switching can
  //happen on the fly. System is wireless by default

  //Start serial connection
  Serial.begin(BAUDRATE);
  delay(250);
  
  //Setup pins:
  pinMode(STEP_PIN_AZ, OUTPUT);
  pinMode(DIR_PIN_AZ, OUTPUT);
  pinMode(EN_PIN_AZ, OUTPUT);
  pinMode(STEP_PIN_EL, OUTPUT);
  pinMode(DIR_PIN_EL, OUTPUT);
  pinMode(EN_PIN_EL, OUTPUT);
  pinMode(AZ_HOMESWITCH, INPUT);
  pinMode(EL_HIGHSTOP, INPUT);
  pinMode(EL_LOWSTOP, INPUT);
  pinMode(POLARITY_SWITCH_1, OUTPUT);
  pinMode(POLARITY_SWITCH_2, OUTPUT);

  //Radio Setup
  radio.begin();
  radio.openReadingPipe(1, addressA);   //Setting the address at which we will receive the data (opposite control)
  radio.openWritingPipe(addressB);      //Setting the address at which we will send the data (opposite control)
  radio.setPALevel(RF24_PA_MIN);       //You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.startListening();
  
  //setup homing switch pins
  pinMode(EL_LOWSTOP, INPUT);
  pinMode(EL_HIGHSTOP, INPUT);
  pinMode(AZ_HOMESWITCH, INPUT);
  sensorPos(); //get initial sensor position

  
  writeOut("Ground Station Active"); //Use this function instead of println()
  
  //setting max step rate of motors:
  AzStepper.setMaxSpeed(800);
  ElStepper.setMaxSpeed(800);

  //Enabling Motors:
  digitalWrite(EN_PIN_AZ, LOW);
  digitalWrite(EN_PIN_EL, LOW);
  
}

void loop() {
  if(wireSelect){
    wirelessLoop();
  }else{
    wiredLoop();
  }
}

void wirelessLoop(){
  //called if the system is operating wirelessly
  //currently no timeout

  //get new target data/command from control side
  
  if(currentLoop == skipLoops){
    while(radio.available()){
      //read in targets
      radio.read(&rxData, sizeof(rxData));
      delay(2);
    }
    
    //update motors, sensorPos called inside motorUpdate()
    
    //Send position data to control computer
    radio.stopListening();
    radio.write(&txData, sizeof(txData));
    txData.command_understood = 0;
    radio.startListening();

    currentLoop = 0;

    if(rxData.command == 10){
      AzStepper.setSpeed(0);
      ElStepper.setSpeed(0);
      AzStepper.runSpeed();
      ElStepper.runSpeed();
      while(true){delay(100);};
      txData.command_understood = 1;
    }

    if(rxData.command == 12){
      polarity_1 = !polarity_1;
      digitalWrite(POLARITY_SWITCH_1, polarity_1);
      txData.command_understood = 1;
    }

    if(rxData.command == 13){
      polarity_2 = !polarity_2;
      digitalWrite(POLARITY_SWITCH_2, polarity_2);
      txData.command_understood = 1;
    }
  }

  
  currentLoop += 1;
  motorUpdate();
}

void wiredLoop() {

  while(Serial.available() > 0) {
    inChar[chCount] = Serial.read();
    chCount += 1;
    delay(1);    
  }

  //Parse the things
  if (chCount != 0 && Serial.available() < 1){
    if(inChar[0] == 'A' && inChar[2] != ' '){
      //Set target command sent
      setTgt();
    }else if(inChar[0] == 'S'){
      //Stop command sent, do nothing
      Serial.println("Stop Command");
    }else if(inChar[0] == 'A' && inChar[2] == ' '){
      //Get command sent
      sendPos();
    }else if(inChar[0] == 'C'){
      calibrate();
    }else if(inChar[0] == 'T'){
      while(HIGH){
        delay(1000);
        Serial.print(digitalRead(AZ_HOMESWITCH));
        Serial.print(digitalRead(EL_HIGHSTOP));
        Serial.println(digitalRead(EL_LOWSTOP));
      }
    }else if(inChar[0] == 'W'){
      //switch to wireless mode
      Serial.println("Switched to wireless mode");
      wireSelect = 1;
    }else{
      Serial.println("Error: Invalid Command");
    }
    memset(inChar, 0, sizeof(inChar)); //Reset inChar array
  }
  motorUpdate();
  chCount = 0;
}

void sensorPos(){
  //No sensors used, going by steps only
  txData.az_position = (AzStepper.currentPosition() * AZ_DEG_PER_STEP) + n; //FIXME
  //Maps current position to between 0 and 90 degrees based on
  //limit switch position. map() is integer, so everything is kept at
  //100 times the required and then divided to form the float txData.el_position
  //ASSUMES the maximum angle achievable is 90 degrees
  txData.el_position = ElStepper.currentPosition() * EL_DEG_PER_STEP; //map function was weird
  
  //If azimuth crossover occurs:
  if(txData.az_position >= 360){
    n -= 360;
  }else if(txData.az_position < 0){
    n += 360;
  }
  
  if(Debug){
    Serial.println(" ");
    Serial.print("Az pos  El pos: ");
    Serial.print(txData.az_position);
    Serial.print("  ");
    Serial.println(txData.el_position);
  }
}

void setTgt(){
  //Sets rxData.az_target and rxData.el_target by parsing input string in inChar
  //start by getting azimuth target:
  char indFrom = 'Z'; //To search from this value
  char indTo = ' '; //Search up to this value
  int carry = 0;
  int valLen = 0;
  
  valLen = getLenBetween(inChar, indFrom, indTo);
  carry = getIndOf(inChar, indFrom);
  rxData.az_target = excludingPlaces(inChar, carry+1, carry+valLen);

  //add bias
  if((rxData.az_target + azBias) > 360){
    rxData.az_target = rxData.az_target + azBias - 360;
  }else if((rxData.az_target - azBias) < 0){
    rxData.az_target = rxData.az_target + azBias + 360;
  }else{
    rxData.az_target = rxData.az_target + azBias;
  }

  indFrom = 'L';
  valLen = getLenBetween(inChar, indFrom, indTo);
  carry = getIndOf(inChar, indFrom);
  rxData.el_target = excludingPlaces(inChar, carry+1, carry+valLen);

  //if((rxData.el_target - elBias) < 0){rxData.el_target = 0;}
  if((rxData.el_target + elBias) > 90){rxData.el_target = 90;}
  else{rxData.el_target = rxData.el_target + elBias;}

  if(Debug){
    Serial.println("Command Received ");
    Serial.println(valLen);
    Serial.print("AZ target is: ");
    Serial.println(rxData.az_target);
    Serial.println(inChar);
  }
}

void motorUpdate(){
  //With this system the pointing is always one second behind required.
  sensorPos();//update sensors
  float dEl = rxData.el_target - txData.el_position;
  float azDist;

  azDist = abs(rxData.az_target - txData.az_position);
  
    //multiple cases for azimuth speed and direction:
    if(azDist > 180 && txData.az_position > 180){
      azDist = (360 - txData.az_position) + rxData.az_target;
      azimuthRate(azDist);
    }else if(abs(azDist) < Tolerance){
      azimuthRate(0);
    }else if(azDist >= 180 && txData.az_position <= 180){
      azDist = txData.az_position + (360 - rxData.az_target);
      azimuthRate(-STEP_LIMIT);
    }else if(azDist < 180 && txData.az_position < rxData.az_target){
      azimuthRate(STEP_LIMIT);
    }else if(azDist < 180 && txData.az_position >= rxData.az_target){
      azimuthRate(-STEP_LIMIT);
    }else{
      Serial.println(azDist);
      Serial.println(txData.az_position);
      runError();
    }

    //setting elevation rate:
    if(getElSensors() != 1){
      elevationRate(0);
    }else if(abs(dEl) > Tolerance){
      elevationRate(dEl);
    }else{
      elevationRate(0);
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
  //lcd.clear();
  //lcd.print("Calibrating...");
  Serial.println("Calibrating");

  //first go to top elevation switch:
  elevationRate(EL_CAL_RATE);
  t = millis();
  while(digitalRead(EL_HIGHSTOP) == LOW){
    dt = millis();
    if(dt - t > EL_CAL_TIMEOUT){Serial.println("timeout");calError();}
    if(digitalRead(EL_LOWSTOP) == HIGH){Serial.println("collision");calError();}
    ElStepper.runSpeed();
  }
  elevationRate(0);
  ElStepper.runSpeed();
  delay(1000);
  //elMaxPoint = analogRead(ELPOT);
  
  //go to bottom elevation switch
  elevationRate(-EL_CAL_RATE);
  t = millis();
  while(digitalRead(EL_LOWSTOP) == LOW){
    dt = millis();
    if(dt - t > EL_CAL_TIMEOUT){calError();}
    if(digitalRead(EL_HIGHSTOP) == HIGH){calError();}
    ElStepper.runSpeed();
  }
  elevationRate(0);
  ElStepper.runSpeed();
  Serial.println("EL Calibrated");
  delay(1000);
  //elMinPoint = analogRead(ELPOT);
  //elevation now finished
  
  //Find azimuth homing switch:
  azimuthRate(AZ_CAL_RATE);
  t = millis();
  while(digitalRead(AZ_HOMESWITCH) == LOW){
    AzStepper.runSpeed();
    dt = millis();
    if(dt - t > AZ_CAL_TIMEOUT){calError();}
  }
  azimuthRate(0);
  AzStepper.runSpeed();
  Serial.println("AZ Calibrated");
  delay(1000);  //pause for effect
}

void elevationRate(float degRate){
  //argument is angular rate in deg/s (positive angle is positive rate)
  //makes elevation motor run at <argument> deg/s
  float stepRate;
  stepRate = degRate / EL_DEG_PER_STEP;
  if(stepRate > STEP_LIMIT){
    ElStepper.setSpeed(STEP_LIMIT); //FIXME same as azimuth rate fixme
  }else if(stepRate < -STEP_LIMIT){
    ElStepper.setSpeed(-STEP_LIMIT);
  }else if(stepRate >= 1){
    ElStepper.setSpeed(STEP_LIMIT);
  }else if(stepRate <= -1){
    ElStepper.setSpeed(-STEP_LIMIT);
  }else{
    ElStepper.setSpeed(0);
  }
  ElStepper.runSpeed(); 
}

void sendPos(){
  
  //Break positions into parts so the sprintf() function can work:
  azInt = int(txData.az_position);                 //Whole part of the txData.az_position float
  azFlt = int((txData.az_position - azInt) * 10);  //Decimal part of txData.az_position float
  elInt = int(txData.el_position);                 //Whole part of the txData.el_position float
  elFlt = int((txData.el_position - elInt) * 10);  //Decimal part of the txData.el_position float

  sprintf(outChar, "AZ%d.%d EL%d.%d", azInt, azFlt, elInt, elFlt);
  Serial.write(outChar);
  Serial.flush(); //holds program while Serial.write sends data
  Serial.println("");
  
}
void azimuthRate(float degRate){
  //argument is angular rate in deg/s (clockwise is positive)
  //makes azimuth motor run at <argument> deg/s  
  float stepRate;
  stepRate = degRate / AZ_DEG_PER_STEP;
  if(stepRate > STEP_LIMIT){
    stepRate = STEP_LIMIT; //FIXME  currently throws an error, could also default to
                           //just running at maximum value in either direction
  }else if(stepRate < -STEP_LIMIT){
    stepRate = -STEP_LIMIT;
  }else{
    AzStepper.setSpeed(0);
  }
  AzStepper.setSpeed(stepRate);
  AzStepper.runSpeed();
} 

void calError(){
  //called when an error occurs in calibration
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
  AzStepper.setSpeed(0);
  ElStepper.setSpeed(0);
  AzStepper.runSpeed();
  ElStepper.runSpeed();
  writeOut("RUN ERROR");
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
  if(digitalRead(EL_LOWSTOP) == LOW || digitalRead(EL_HIGHSTOP) == LOW){
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

void writeOut(char chOut[32]){
  //Writes to Serial monitor if wired connection, otherwise dumps input
  if(!wireSelect){
    Serial.println(chOut);
  }
}
