
/* Copyright (C) 2016 - INTUITIVE ROBOTICS, INC. - All Rights Reserved
 * You're not allowed to use, distribute and modify this code under the
 * terms of the Inuitive Robotics, Inc. license, unless you've signed an
 * NDA with Intuitive Robotics, Inc.
 *
 * Ello by Intuitive Robotics, Inc. Copyright (C) 2016 
 * Firmware Developers: Hassan Murad - hassanmurad93@gmail.com | Vivek Vyas - vvyas@sfu.ca
 */



// LIBRARIES 
#include <SoftwareSerial.h>
#include <Servo.h>
#include <AFMotor.h>
#include <Wire.h>
#include "pitches.h"



// DEFINITIONS 
AF_DCMotor          motor(1);//DC motor
#define vrLED       2     //LED to indicate VR -ElloEllo is detected
#define organicsLED 3     //LED to indicate VR -ElloEllo is detected
#define landfillLED 4     //LED to indicate VR -ElloEllo is detected
#define trigpin     5     //set trigpin - wave
#define echopin     6     //set echopin - wave
#define trigpin2    9     //set trigpin2 - capacity
#define echopin2    10    //set echopin2 - capacity
#define bluetoothTx 11    //define bluetooth TX
#define bluetoothRx 12    //define bluetooth RX
#define binHeight   64.5  //bin height
#define holdTime    5000  //5 seconds
#define closeTime   3000  //3 seconds
#define openTime    2500  //2.5 seconds
#define debug       1     //If 1 - will print status of bin capacity, distance and etc.
#define plotting    0     //Enable (1) If plotting script is to be used
#define blueSend    1     //1 - Send Capacity data via bluetooth, 0 - Don't Send via bluetooth



// DECLARATIONS 
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx); //enables desired pins to be fully capable for serial communication

int duration, waveDistance;//declare variable for ultrasonic sensor to detect hand
int duration2, capDistance, capacityDistance;//declare variable for unltrasonic sensor to detect capacity
int servoPosition = 0; int capacity = 0;
int state = 0;         int prevState = 0;

// notes in the melody:
int melody[] = {NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4};
// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {4, 8, 8, 4, 4, 4, 4, 4};


// SETUP PROCEDURE
void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600); //sets up bluetooth serial connection to android
  
  Wire.begin(5);
  Wire.onReceive(receiveEvent);
  
  pinMode(trigpin,  OUTPUT);
  pinMode(echopin,  INPUT);
  pinMode(trigpin2, OUTPUT);
  pinMode(echopin2, INPUT);
  pinMode(vrLED,  OUTPUT);
  pinMode(organicsLED,  OUTPUT);
  pinMode(landfillLED,  OUTPUT);
  //pinMode(8, OUTPUT);

  digitalWrite(vrLED,LOW);
  digitalWrite(organicsLED,LOW);
  digitalWrite(landfillLED,LOW);
  motor.setSpeed(0);
}



// MAIN LOOP
void loop() {

  //Get Ultrasonic Distance - Wave & Capacity Sensors
  waveDistance = getDistance(trigpin, echopin); 
  capDistance = getDistance(trigpin2, echopin2);

  //Check for HAND GUESTURE | VR COMMAND | BLUETOOTH MSGS
  if(abs(waveDistance) <= 3 && abs(waveDistance)!=4){digitalWrite(organicsLED,LOW); digitalWrite(landfillLED,LOW); motionSequence(); }
  else if(state == 1 && state!=prevState){digitalWrite(organicsLED,LOW); digitalWrite(landfillLED,LOW); openLid(); }                     //Open Lid (received a H) 
  else if(state == 2 && state!=prevState){digitalWrite(organicsLED,LOW); digitalWrite(landfillLED,LOW); closeLid(); }                    //Close Lid (received a L)
  else if(state == 3 && state!=prevState){digitalWrite(landfillLED,LOW); digitalWrite(organicsLED,LOW); motionSequence(); digitalWrite(landfillLED,LOW); } //Mixed Containers (received a M)
  else if(state == 4 && state!=prevState){digitalWrite(organicsLED,HIGH); digitalWrite(landfillLED,LOW);}                  //Organics (received a O)
  else if(state == 5 && state!=prevState){digitalWrite(landfillLED,HIGH); digitalWrite(organicsLED,LOW);}                  //Garbage/Landfill (received a G)
  else if(state == 6 && state!=prevState){digitalWrite(vrLED,HIGH); /*loginTone();*/}                                          //Turn ON VR LED
  else if(state == 7 && state!=prevState){digitalWrite(vrLED,LOW); digitalWrite(organicsLED,LOW); digitalWrite(landfillLED,LOW);}       //Sleep - Turn OFF VR LED
  else if (bluetooth.available() > 0){receiveData();}                                                                     //Check for bluetooth msg
      
  capacityDistance = saturate(map(capDistance, binHeight, 0, 0, 100));
  prevState = state;
 
  if (debug == 1){debugger();}  
  if (plotting == 1){plotter();}      
  if (blueSend == 1){bluetoothSend();}
}



// FUNCTIONS


//Objective: Prints out to serial monitor for debugging purposes.
void debugger(){
  Serial.print("Ultrasonic Distance (cm): ");   //Print distance unit cm
  Serial.println(waveDistance);                     //Distance
  Serial.print("Capacity Distance: ");          //Print servo position
  Serial.println(capDistance);                    //Degrees
  Serial.print("Bin Capacity %: ");             //Print Bin Capacity
  Serial.println(capacityDistance); 
  Serial.print("State: ");                      //Print Bin Capacity
  Serial.println(state);       
}


//Objective: Prints out to serial monitor for debugging purposes.
void loginTone(){
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 8; thisNote++) {

    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(8, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(8);
  }  
}


//Objective: Sends appropriate data to Serial Monitor for python script to plot.
void plotter(){
  Serial.print(capacityDistance);
  Serial.print(" ");
  Serial.print(waveDistance);
  Serial.print("\n");       
}



//Objective: Sends appropriate data through bluetooth to the App using the format: #data~
//           # starting symbol, ~ ending delimeter - extracts all data between # and ~
void bluetoothSend(){
  bluetooth.write('#');
  bluetooth.print(capacityDistance);
  bluetooth.write('~');  
}



//Objective: Saturate bin capacity between 0-100%
float saturate(float cap){
  if      (cap>100){cap = 100;}
  else if (cap<0)  {cap = 0;}
  return cap;
}




//Objective: Receives data from bluetooth - Opens or Closes Lid
void receiveData(){
  int inbyte = bluetooth.read(); //reads from th bluetooth and writes to USB serial
  Serial.println(inbyte);
  
  if (inbyte == '0'){
    if (debug == 1) {Serial.println ("Closing Lid.");}
    closeLid();   //Close Lid
  }
  if (inbyte == '1'){
    if (debug == 1) {Serial.println ("Opening Lid.");}
    openLid();    //Open Lid
  } 
}



//Objective: Open the lid, hold for awhile and then close the lid.
void motionSequence(){
  //open lid and then hold for awhile before closing
  motor.run(BACKWARD);
  motor.setSpeed(255); 
  delay(openTime);
  motor.run(RELEASE);
  motor.setSpeed(0);
  delay(holdTime);
  motor.run(FORWARD);
  motor.setSpeed(255);  
  delay(closeTime);
  motor.run(RELEASE);
  motor.setSpeed(0);
}



//Objective: Close Lid.
void closeLid(){
  //close lid
  motor.run(FORWARD);
  motor.setSpeed(255);
  delay(closeTime);
  //stop motor
  motor.setSpeed(0);
  motor.run(RELEASE);
  delay(1000);
}



//Objective: Open Lid.
void openLid(){
  //open lid
  motor.run(BACKWARD);
  motor.setSpeed(255);  
  delay(openTime);
  //stop motor
  motor.setSpeed(0);
  motor.run(RELEASE);
  delay(1000);
}



//Objective: Interrupt for I2C bus - Check for VR Commands.
void receiveEvent(int howMany){
  while(Wire.available()){
    char c = Wire.read();
    if     (c == 'H'){state = 1;}    //Open lid - Whistle
    else if(c == 'L'){state = 2;}    //Close lid - Close
    else if(c == 'M'){state = 3;}    //Mixed containers - Paper Cups
    else if(c == 'O'){state = 4;}    //Organics - Apple
    else if(c == 'G'){state = 5;}    //Garbage - Cigarrette
    else if(c == '1'){state = 6;}    //Group 1 entered - ElloEllo Detected, Turn VR LED ON
    else if(c == '0'){state = 7; if(debug == 1){Serial.println("****** LED OFF! ********\n");}}    //Not in Group 1 yet - Keep VR LED off
    if (debug == 1){ Serial.print("C: ");Serial.println(c); Serial.print("State: "); Serial.println(state);}
  }
}



//Objective: Get distance from an Ultrasonic Sensor.
float getDistance(int triggerPin, int echoPin){
  digitalWrite(triggerPin,HIGH);
  _delay_ms(50);
  digitalWrite(triggerPin, LOW);
  int duration = pulseIn(echoPin, HIGH);
  int dist = (duration/2)/29.1;
  return dist;
}





