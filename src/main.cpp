/*
 Project: SRADio Test Code
 Created: 12/29/2017 12:33:25 PM
 Authors: Tim Vrakas
          Joan Creus-Costa

 Description:
 GFSK (2-way) radio communication with custom SRADio Hardware.

 Credits:
 Femtoloon and Iskender Kushan's SSTV code.
 Aria Tedjarati and Joan Creus-Costa for VBRF

 Status:
 12/29/2017 Rewrite Library

 TODO as of 12/29/2017:
 -Add RX code
 -Test With new Hardware
 -Build packet parser
*/

#include <Arduino.h>
#include "SRADio.h"

uint8_t message_length = MAX_MSG_LENGTH;
uint8_t latest_message[MAX_MSG_LENGTH] = {0};

SRADio SRADio1;

int main()
{

  Serial.begin(115200);
  Serial.println("SRADio Jan 2018");

  SRADio1.configureRF();

  delay(3000);

  uint32_t sendRadioTimer = millis() + 1000;
  int i = 0;

  while (true)
  {
    //look for new data here.
    if (millis() > sendRadioTimer)
    {
      //generate some data paterns!
      i = (i + 1) % MAX_MSG_LENGTH;
      latest_message[i]++;

      SRADio1.encode_and_transmit(latest_message, message_length);
      sendRadioTimer = millis() + 1000;
    }
  }
}
