
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
//#include <Wire.h>
//#include "pitches.h"



// DEFINITIONS 
//AF_DCMotor          motor(1);//DC motor
#define vrLED       2     //LED to indicate VR -ElloEllo is detected
#define organicsLED 3     //LED to indicate VR -ElloEllo is detected
#define landfillLED 4     //LED to indicate VR -ElloEllo is detected
#define trigpin     10     //set trigpin - wave
#define echopin     11     //set echopin - wave
#define trigpin2    5     //set trigpin2 - capacity
#define echopin2    6    //set echopin2 - capacity
#define bluetoothTx 11    //define bluetooth TX
#define bluetoothRx 12    //define bluetooth RX
#define binHeight   64.5  //bin height
#define holdTime    5000  //5 seconds
#define closeTime   3000  //3 seconds
#define openTime    2500  //2.5 seconds
#define debug       1     //If 1 - will print status of bin capacity, distance and etc.
#define plotting    0     //Enable (1) If plotting script is to be used
#define blueSend    0     //1 - Send Capacity data via bluetooth, 0 - Don't Send via bluetooth



// DECLARATIONS 
//SoftwareSerial bluetooth(bluetoothTx, bluetoothRx); //enables desired pins to be fully capable for serial communication
Servo myservo;  // create servo object to control a servo

int duration, waveDistance;//declare variable for ultrasonic sensor to detect hand
int duration2, capDistance, capacityDistance;//declare variable for unltrasonic sensor to detect capacity
int servoPosition = 0; int capacity = 0;
int state = 0;         int prevState = 0;
int pos = 0;

// notes in the melody:
//int melody[] = {NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4};
//// note durations: 4 = quarter note, 8 = eighth note, etc.:
//int noteDurations[] = {4, 8, 8, 4, 4, 4, 4, 4};


// SETUP PROCEDURE
void setup() {
  Serial.begin(9600);
 // bluetooth.begin(9600); //sets up bluetooth serial connection to android
  
//  Wire.begin(5);
  //Wire.onReceive(receiveEvent);
  
  pinMode(trigpin,  OUTPUT);
  pinMode(echopin,  INPUT);
  pinMode(trigpin2, OUTPUT);
  pinMode(echopin2, INPUT);
  myservo.attach(9);
 // pinMode(vrLED,  OUTPUT);
 // pinMode(organicsLED,  OUTPUT);
 // pinMode(landfillLED,  OUTPUT);
  //pinMode(8, OUTPUT);

  //digitalWrite(vrLED,LOW);
  //digitalWrite(organicsLED,LOW);
  //digitalWrite(landfillLED,LOW);
  //motor.setSpeed(0);
}



// MAIN LOOP
void loop() {

  //Get Ultrasonic Distance - Wave & Capacity Sensors
  waveDistance = getDistance(trigpin, echopin); 
  capDistance = getDistance(trigpin2, echopin2);

  //Check for HAND GUESTURE | VR COMMAND | BLUETOOTH MSGS
  if(abs(waveDistance) <= 3 && abs(waveDistance)!=4){
      motionSequence(); 
    }     //Sleep - Turn OFF VR LED                                                                    //Check for bluetooth msg
      
  capacityDistance = saturate(map(capDistance, binHeight, 0, 0, 100));
  prevState = state;
 
  if (debug == 1){debugger();}  
  if (plotting == 1){plotter();}      
  //if (blueSend == 1){bluetoothSend();}
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
}



//Objective: Sends appropriate data to Serial Monitor for python script to plot.
void plotter(){
  Serial.print(capacityDistance);
  Serial.print(" ");
  Serial.print(waveDistance);
  Serial.print("\n");       
}




//Objective: Saturate bin capacity between 0-100%
float saturate(float cap){
  if      (cap>100){cap = 100;}
  else if (cap<0)  {cap = 0;}
  return cap;
}





//Objective: Open the lid, hold for awhile and then close the lid.
void motionSequence(){
  //open lid and then hold for awhile before closing
  for (pos = 220; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(1);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 0; pos <= 220; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(1);                       // waits 15ms for the servo to reach the position
  }
  myservo.write(0);
  delay(2000);
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





