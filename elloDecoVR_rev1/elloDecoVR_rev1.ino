/* Copyright (C) 2016 - INTUITIVE ROBOTICS, INC. - All Rights Reserved
 * You're not allowed to use, distribute and modify this code under the
 * terms of the Inuitive Robotics, Inc. license, unless you've signed an
 * NDA with Intuitive Robotics, Inc.
 *
 * Ello by Intuitive Robotics, Inc. Copyright (C) 2016 
 * Firmware Developers: Hassan Murad - hassanmurad93@gmail.com | Vivek Vyas - vvyas@sfu.ca
 */

#include <Wire.h>
#include "Arduino.h"
#if !defined(SERIAL_PORT_MONITOR)
  #error "Arduino version not supported. Please update your IDE to the latest version."
#endif

#if defined(SERIAL_PORT_USBVIRTUAL)
  // Shield Jumper on HW (for Leonardo and Due)
  #define port SERIAL_PORT_HARDWARE
  #define pcSerial SERIAL_PORT_USBVIRTUAL
#else
  // Shield Jumper on SW (using pins 12/13 or 8/9 as RX/TX)
  #include "SoftwareSerial.h"
  SoftwareSerial port(12, 13);
  #define pcSerial SERIAL_PORT_MONITOR
#endif

#include "EasyVR.h"

EasyVR easyvr(port);

//Groups and Commands
enum Groups
{
  GROUP_0  = 0,
  GROUP_1  = 1,
};

enum Group0 
{
  G0_ELLOELLO = 0,
};

enum Group1 
{
  G1_APPLE = 0,
  G1_CIGGERATE = 1,
  G1_PAPERCUP = 2,
  G1_OPEN = 3,
  G1_CLOSE = 4,
  G1_PLASTIC = 5,
  G1_WHISTLE = 6,
  G1_CLAP = 7,
};


int8_t group, idx;
int blueLED = 7;
int redLED = 8;
int yellowLED = 9;

void setup()
{
  Wire.begin(); //Setup I2C Bus
  
  //setup LEDs
  pinMode (redLED, OUTPUT);
  digitalWrite (redLED, LOW);
  pinMode (blueLED, OUTPUT);
  digitalWrite (blueLED, LOW);
  pinMode (yellowLED, OUTPUT);
  digitalWrite (yellowLED, LOW);
  
  // setup PC serial port
  pcSerial.begin(9600);

  // bridge mode?
  int mode = easyvr.bridgeRequested(pcSerial);
  switch (mode)
  {
  case EasyVR::BRIDGE_NONE:
    // setup EasyVR serial port
    port.begin(9600);
    // run normally
    pcSerial.println(F("---"));
    pcSerial.println(F("Bridge not started!"));
    break;
    
  case EasyVR::BRIDGE_NORMAL:
    // setup EasyVR serial port (low speed)
    port.begin(9600);
    // soft-connect the two serial ports (PC and EasyVR)
    easyvr.bridgeLoop(pcSerial);
    // resume normally if aborted
    pcSerial.println(F("---"));
    pcSerial.println(F("Bridge connection aborted!"));
    break;
    
  case EasyVR::BRIDGE_BOOT:
    // setup EasyVR serial port (high speed)
    port.begin(115200);
    // soft-connect the two serial ports (PC and EasyVR)
    easyvr.bridgeLoop(pcSerial);
    // resume normally if aborted
    pcSerial.println(F("---"));
    pcSerial.println(F("Bridge connection aborted!"));
    break;
  }

  while (!easyvr.detect())
  {
    Serial.println("EasyVR not detected!");
    delay(1000);
  }

  easyvr.setPinOutput(EasyVR::IO1, LOW);
  Serial.println("EasyVR detected!");
  easyvr.setTimeout(5);
  easyvr.setLanguage(0);

  group = EasyVR::TRIGGER; //<-- start group (customize)
}

void action();

void loop()
{
  if (easyvr.getID() < EasyVR::EASYVR3)
    easyvr.setPinOutput(EasyVR::IO1, HIGH); // LED on (listening)

  Serial.print("Say a command in Group ");
  Serial.println(group);
  easyvr.recognizeCommand(group);

  do
  {
    // can do some processing while waiting for a spoken command
  }
  while (!easyvr.hasFinished());
  
  if (easyvr.getID() < EasyVR::EASYVR3)
    easyvr.setPinOutput(EasyVR::IO1, LOW); // LED off

  idx = easyvr.getWord();
  if (idx >= 0)
  {
    // built-in trigger (ROBOT)
    // group = GROUP_X; <-- jump to another group X
    return;
  }
  idx = easyvr.getCommand();
  if (idx >= 0)
  {
    // print debug message
    uint8_t train = 0;
    char name[32];
    Serial.print("Command: ");
    Serial.print(idx);
    if (easyvr.dumpCommand(group, idx, name, train))
    {
      Serial.print(" = ");
      Serial.println(name);
    }
    else
      Serial.println();
	// beep
    easyvr.playSound(0, EasyVR::VOL_FULL);
    // perform some action
    action();
  }
  else // errors or timeout
  {
    if (easyvr.isTimeout())
      Serial.println("Timed out, try again...");
    int16_t err = easyvr.getError();
    if (err >= 0)
    {
      Serial.print("Error ");
      Serial.println(err, HEX);
    }
  }
}

void action()
{
    switch (group)
    {
    case GROUP_0:
      switch (idx)
      {
      case G0_ELLOELLO:
        // write your action code here
         group = GROUP_1; //or jump to another group X for composite commands
        break;
      }
      break;
    case GROUP_1:
      switch (idx)
      {
        case G1_APPLE:
           Serial.println ("Say Apple");
          // write your action code here
           Wire.beginTransmission(5);
           Wire.write('O');
           Wire.endTransmission();
           break;
        case G1_CIGGERATE:
          // write your action code here
          Wire.beginTransmission(5);
          Wire.write('G');
          Wire.endTransmission();
          break;
        case G1_PAPERCUP:
          // write your action code here
           Wire.beginTransmission(5);
           Wire.write('M');
           Wire.endTransmission();
           break;
        case G1_OPEN:
          // write your action code here
          Wire.beginTransmission(5);
          Wire.write('H');
          Wire.endTransmission();
          break;
          
        case G1_CLOSE:
          // write your action code here
           Wire.beginTransmission(5);
           Wire.write('L');
           Wire.endTransmission();
          // group = GROUP_X; <-- or jump to another group X for composite commands
           break;
        case G1_PLASTIC:
          // write your action code here
          // group = GROUP_X; <-- or jump to another group X for composite command
           Wire.beginTransmission(5);
           Wire.write('M');
           Wire.endTransmission();
           break;
        case G1_WHISTLE:
          // write your action code here
          // group = GROUP_X; <-- or jump to another group X for composite commands
           Wire.beginTransmission(5);
           Wire.write('H');
           Wire.endTransmission();
           break;
  
        case G1_CLAP:
          // write your action code here
          // group = GROUP_X; <-- or jump to another group X for composite commands
           Wire.beginTransmission(5);
           Wire.write('L');
           Wire.endTransmission();
           break;
      }
      break;
    }
}
