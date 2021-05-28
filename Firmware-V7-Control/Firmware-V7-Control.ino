/*
Ground station ground station pointing system
controller firmware written for Carleton University Designated Study.
Author: August A. Lear
Student #: 101075941
Current Version Date: 2021-04-21
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

THIS IS WIRELESS
USE IN TANDEM WITH Firmware-V7-Station.ino
THIS TO BE UPLOADED TO NANO, CONNECTED TO CONTROL COMPUTER
*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <LiquidCrystal.h>
#include <AccelStepper.h> //INSTALL WITH LIBRARY MANAGER
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//DATA CONSTANT SETUP
#define BAUDRATE 9600 //BAUD rate of serial connection
#define BUFFERLEN 64 //Length of data array sent and received
#define CYCLEWAIT 100//delay at end of main loop (millis)

//Define pins for the LCD
const int rs = 3, e = 2, d4 = 4, d5 = 5, d6 = 6, d7 = 7;  //FIXME (replace with I2C enabled LCD)
LiquidCrystal lcd(rs, e, d4, d5, d6, d7); //instatiate lcd object

//Instantiate Wireless transciever:
RF24 radio(9,10); //CE, CSN

//MISC DEFINITIONS
bool err = 0; //tracks errors
bool Debug = 0; //MAKE 1 FOR DEBUG, 0 FOR NORMAL(DEV)
int azBias = 0; //pointing bias for az axis
int elBias = 0; //pointing bias for el axis

int chCount = 0;  //data reception counter
int azInt = 0, azFlt = 0, elInt = 0, elFlt = 0; //For sprintf
int n = 0; //counter for azimuth crossover direction

//RADIO:
const byte addressA[6] = "00001";     //Address byte to write to
const byte addressB[6] = "00002";     //Address byte to read from
int timeoutCount = 0;                 //Number of missed packets
int timeoutMax = 100;                  //Number of allowed missed packets

struct control_tx {
  //el_target contains the elevation target being sent from GPredict
  float el_target;
  //az_target contains the elevation target being sent from GPredict
  float az_target;
  //command contains a byte, which defines the command.
  int command;
};
struct control_rx {
  //el_position contains the elevation position being received from the station
  float el_position;
  //az_position contains the azimuth position being received from the station
  float az_position;

  bool command_understood;
};

control_tx txData;
control_rx rxData;

char inChar[BUFFERLEN]; //Array into which data is received from the Serial buffer
char outChar[BUFFERLEN];  //Array to be written to the serial buffer

//DEBUGGING
unsigned long timeFlag = 0;
unsigned long timeDif = 0;

void setup() {
  
  Serial.begin(BAUDRATE);
  //Radio Setup
  radio.begin();
  radio.openReadingPipe(0, addressB);   //Setting the address at which we will receive the data
  radio.openWritingPipe(addressA);      //Setting the address at which we will send the data
  radio.setPALevel(RF24_PA_MIN);       //You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.startListening();
  
  /*
  //Begin communication to 16-position, 2 line lcd
  lcd.begin(16,2);
  lcd.noAutoscroll(); //prevents LCD from autoscrolling
  */
  Serial.println("Control Transciever Active");
}

void loop() {
  
  //Read new byte from Serial buffer, then add that byte to the inChar array
  while(Serial.available() > 0) {
    inChar[chCount] = Serial.read();
    chCount += 1;
    delay(1);
  }
  //If radio detects something, read that in as new position:
  while(radio.available()){
    radio.read(&rxData, sizeof(rxData));
    //delay(1); //maybe need this later
    timeoutCount = 0;
  }

  //If Serial data is recieved:

  if (chCount != 0 && Serial.available() < 1){
    if (rxData.command_understood)
      txData.command = 0;
    if(inChar[0] == 'A' && inChar[2] != ' '){ 
      //Command to set new target
      setTgt();
    }else if(inChar[0] == 'S'){ 
      //Flush block
      flushBlock();
    }else if(inChar[0] == 'A' && inChar[2] == ' '){ 
      //Get Position Command
      getPos();
    }else if(inChar[0] == 'K'){
      //Home-brew function. Call to initiate calibration
      txData.command = 10;
      //calibrate();
    }else if(inChar[0] == 'W'){
      Serial.println("Setting station to wired mode");
      //Home-brew function. set command to 7, switches station to wired mode
      txData.command = 7;
      radio.stopListening();
      radio.write(&txData, sizeof(txData));
      radio.startListening();
      txData.command = 0;
    }
    // OTHER FUNCTIONS:
    else if (inChar[0] == 'M')
    { //TOGGLE POLARITY SWITCH 1
      txData.command = 12;
    }
    else if (inChar[0] == 'N')
    { //TOGGLE POLARITY SWITCH 2
      txData.command = 13;
    }
    else{
      Serial.println("ERROR: Invalid Command");
    }
    memset(inChar, 0, sizeof(inChar)); //Reset inChar array
  }
  
  //Transmit output packet
  radio.stopListening();
  radio.write(&txData, sizeof(txData));
  radio.startListening();
  
  chCount = 0;  //reset counter
  timeoutCount += 1; //count up the number of missed packets
  if(timeoutCount > timeoutMax){
    timeOut();
    timeoutCount = 0;
  }
  //lcdUpdate();  //update LCD
  delay(1);
}

void lcdUpdate(){
  //Updates lcd with values of target and position
  char lcdOut1[16];
  char lcdOut2[16];
  
  sprintf(lcdOut1, "PA:%d|PE:%d", int(rxData.az_position), int(rxData.el_position)); //FIXME? 
  sprintf(lcdOut2, "TA:%d|TE:%d", int(txData.az_target), int(txData.el_target)); //FIXME ?

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(lcdOut1);//writing from top-left of lcd
  lcd.setCursor(0,1);//writing from bottom-left of LCD
  lcd.print(lcdOut2); 
}

void setTgt(){
  //Sets txData.az_target and txData.el_target by parsing input string in inChar
  //start by getting azimuth target:
  char indFrom = 'Z'; //To search from this value
  char indTo = ' '; //Search up to this value
  int carry = 0;
  int valLen = 0;
  
  valLen = getLenBetween(inChar, indFrom, indTo);
  carry = getIndOf(inChar, indFrom);
  txData.az_target = excludingPlaces(inChar, carry+1, carry+valLen);

  //add bias
  if((txData.az_target + azBias) > 360){
    txData.az_target = txData.az_target + azBias - 360;
  }else if((txData.az_target - azBias) < 0){
    txData.az_target = txData.az_target + azBias + 360;
  }else{
    txData.az_target = txData.az_target + azBias;
  }

  indFrom = 'L';
  valLen = getLenBetween(inChar, indFrom, indTo);
  carry = getIndOf(inChar, indFrom);
  txData.el_target = excludingPlaces(inChar, carry+1, carry+valLen);

  if((txData.el_target - elBias) < 0){txData.el_target = 0;}
  if((txData.el_target + elBias) > 90){txData.el_target = 90;}
  else{txData.el_target = txData.el_target + elBias;}

  if(Debug){
    Serial.println("Command Received ");
    Serial.println(valLen);
    Serial.print("AZ target is: ");
    Serial.println(txData.az_target);
    Serial.println(inChar);
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

void timeOut(){
  //called when the number of missed packets exceeds the threshold
  //Serial.println("TIMEOUT ERROR");
  //Might need to make this do more later
}

void flushBlock(){
  //called when the first character in the serial input is "S"
  //currently delays 1ms and does nothing else
  delay(1);
}

void calibrate(){
  //called when the calibrate feature is required.
  //currently does nothing
  delay(1);
}

void getPos(){
  //Called when the get position command is received in the serial input
  
  //Break positions into parts so the sprintf() function can work:
  azInt = int(rxData.az_position);                 //Whole part of the rxData.az_position float
  azFlt = int((rxData.az_position - azInt) * 10);  //Decimal part of rxData.az_position float
  elInt = int(rxData.el_position);                 //Whole part of the rxData.el_position float
  elFlt = int((rxData.el_position - elInt) * 10);  //Decimal part of the rxData.el_position float

  sprintf(outChar, "AZ%d.%d EL%d.%d", azInt, azFlt, elInt, elFlt);
  Serial.write(outChar);
  Serial.println("");
  Serial.flush();

}
