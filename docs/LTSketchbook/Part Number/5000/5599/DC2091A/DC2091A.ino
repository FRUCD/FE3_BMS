/*!
Linear Technology DC2091A Demonstration Board.
LTC5599: 130 to 1300MHz IQ Modulator.

@verbatim

NOTES
  Setup:
   Set the terminal baud rate to 115200 and select the newline terminator.

USER INPUT DATA FORMAT:
 decimal : 1024
 hex     : 0x400
 octal   : 02000  (leading 0 "zero")
 binary  : B10000000000
 float   : 1024.0

@endverbatim

http://www.linear.com/product/LTC5599

http://www.linear.com/product/LTC5599#demoboards

REVISION HISTORY
$Revision: 3018 $
$Date: 2014-12-01 15:53:20 -0800 (Mon, 01 Dec 2014) $

Copyright (c) 2013, Linear Technology Corp.(LTC)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of Linear Technology Corp.

The Linear Technology Linduino is not affiliated with the official Arduino team.
However, the Linduino is only possible because of the Arduino team's commitment
to the open-source community.  Please, visit http://www.arduino.cc and
http://store.arduino.cc , and consider a purchase that will help fund their
ongoing work.
*/

/*! @file
    @ingroup LTC5599
*/

#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "UserInterface.h"
#include "LT_I2C.h"
#include "QuikEval_EEPROM.h"
#include "LTC5599.h"
#include <SPI.h>
#include <Wire.h>

// Function Declaration
void print_title();                                         // Print the title block
void print_prompt();                                        // Prompt the user for an input command
void print_user_command(uint8_t menu);                      // Display selected differential channels

void menu_1_read_register();
void menu_2_write_register();

//Global variables
uint8_t address;
uint8_t rw;
                                              
//! Initialize Linduino
void setup()
{
  char demo_name[]="DC2091";      //!< Demo Board Name stored in QuikEval EEPROM 
  quikeval_I2C_init();           // Configure the EEPROM I2C port for 100kHz
  quikeval_SPI_init();           // Configure the spi port for 4MHz SCK
  quikeval_SPI_connect();        // Connect SPI to main data port
  Serial.begin(115200);          // Initialize the serial port to the PC
  print_title();
  print_prompt();
}


//! Repeats Linduino loop
void loop()
{
  uint16_t user_command;

    if (Serial.available())
    {
      user_command = read_int();        // Read the user command
      if (user_command != 'm')
        Serial.println(user_command);   // Prints the user command to com port
      switch (user_command)
      {
        case 1:
          menu_1_read_register();
          break;
        case 2:
          menu_2_write_register();
          break;
        default:
          Serial.println("Invalid Option");
          break;
      }
      Serial.println();
      print_prompt();
    }
}


// Function Definitions
//! Read register
//! @ return void
void menu_1_read_register()
{
  uint8_t write_byte = 0;
  uint8_t readback_byte;
//  uint8_t address;

  Serial.println();
  Serial.print(F("Enter the desired register address in HEX (precede with '0x'): "));
  address = read_int();
  Serial.println(address, HEX);
  Serial.println(F(""));
 
  rw = LTC5599_READ;  //set to read
  LTC5599_write_read(LTC5599_CS, address, rw, write_byte, &readback_byte);
  
  Serial.println(F(""));
  Serial.print(F("Readback byte (in hex): "));
  Serial.println(readback_byte, HEX); 
  Serial.print(F("Readback byte (in binary): "));
  Serial.println(readback_byte, BIN); 
}


//! Write to register & readback
//! @ return void
void menu_2_write_register()
{
  uint8_t write_byte;
  uint8_t readback_byte;
//  uint8_t address;

  Serial.println();
  Serial.print(F("Enter the desired register address in HEX (precede with '0x'): "));
  address = read_int();
  Serial.println(address, HEX);
  Serial.println(F(""));
 
  Serial.println();
  Serial.print(F("Enter the desired register byte in HEX: "));
  write_byte = read_int();
  Serial.println(write_byte, HEX);
  Serial.println(F(""));  

  rw = LTC5599_WRITE;  //set to write 
  LTC5599_write_read(LTC5599_CS, address, rw, write_byte, &readback_byte);
  
  rw = LTC5599_READ;  //set to read
  LTC5599_write_read(LTC5599_CS, address, rw, write_byte, &readback_byte);
    
  Serial.print(F("Readback byte (in hex): "));
  Serial.println(readback_byte, HEX); 
  Serial.print(F("Readback byte (in binary): "));
  Serial.println(readback_byte, BIN);
}


//! Prints the title block when program first starts.
void print_title()
{
  Serial.println();
  Serial.println(F("*****************************************************************"));
  Serial.println(F("* DC2091A Demonstration Program                                 *"));
  Serial.println(F("*                                                               *"));
  Serial.println(F("* This program demonstrates how to send data                    *"));
  Serial.println(F("* to the LTC5599 IQ modulator.                                  *"));
  Serial.println(F("*                                                               *"));
  Serial.println(F("* Set the baud rate to 115200 and select the newline terminator.*"));
  Serial.println(F("*                                                               *"));
  Serial.println(F("*****************************************************************"));
}


//! Prints main menu.
void print_prompt()
{
  Serial.println(F(" 1-Read Register"));
  Serial.println(F(" 2-Write Register"));
  Serial.println();
  Serial.print(F("Enter a command: "));
}

