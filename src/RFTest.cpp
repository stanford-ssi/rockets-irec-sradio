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
 -Test With new Hardware
 -Build packet parser
*/

#include <Arduino.h>
#include "SRADio.h"
#include "hermes.h"
#include "hermes_irec.h"

uint8_t tx_message_length = MAX_MSG_LENGTH;
uint8_t tx_message[MAX_MSG_LENGTH] = {0};

uint8_t rx_message[MAX_MSG_LENGTH] = {0};

SRADio SRADio1;

int main()
{

  Hermes skybass_interface(Serial1);

  skybass_data_t data;
  data.latitude = 5.0;
  skybass_interface.send(data);

  Serial.begin(115200);
  Serial.println("SRADio Jan 2018");

  SRADio1.configureRF();

  delay(3000);

  uint32_t sendRadioTimer = millis() + 1000;
  int i = 0;

  while (true)
  {

    if(SRADio1.tryToRX(rx_message) == 1){
      Serial.print("Got Frame: ");
      Serial.write(rx_message, MAX_MSG_LENGTH);
      Serial.println();
      Serial.printf("RSSI: %u", SRADio1.getRSSI());
      Serial.println("encoded data");
  for (int i = 0; i < MAX_MSG_LENGTH; i++)
  {
    uint8_t k = rx_message[i];
    Serial.print(k);
    if (k < 10)
      Serial.print(" ");
    if (k < 100)
      Serial.print(" ");
    Serial.print(" ");
  }
  Serial.println();
    }


    /* if (millis() > sendRadioTimer)
    {
      //generate some data paterns!
      i = (i + 1) % MAX_MSG_LENGTH;
      tx_message[i]++;

      SRADio1.encode_and_transmit(tx_message, tx_message_length);
      sendRadioTimer = millis() + 1000;
    } */
  }
}

