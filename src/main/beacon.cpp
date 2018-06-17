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

uint8_t tx_message_length = MAX_MSG_LENGTH;
uint8_t tx_message[MAX_MSG_LENGTH];
uint8_t rx_message[MAX_MSG_LENGTH];

SRADio SRADio1;

int main()
{
    Serial.begin(115200);
    pinMode(4,OUTPUT);
    digitalWrite(4,HIGH);

    for (int i = 0; i < 10; i++)
    {
        delay(100);
        Serial.printf("delay%i\r\n", i);
    }
    Serial.println("SRADio Jan 2018");

    SRADio1.configureRF();

    for (int i = 0; i < MAX_MSG_LENGTH; i++)
    {
        tx_message[i] = 255;
        Serial.print(" " + tx_message[i]);
    }
    Serial.println();

    for (int i = 10; i < 20; i++)
    {
        delay(100);
        Serial.printf("delay%i\r\n", i);
    }

    uint32_t pktTimer = millis(); //timer between tx packets
    uint32_t listenTimer = millis();         //time of last rx'd packet
    uint32_t sendTimer = millis();             //time we started beaconing
    bool sending = false;
    int i = 0;

    while (true)
    {
        if (sending)
        {
            if (millis() > pktTimer + 50)
            {
                Serial.println("tryin");
                //generate some data paterns

                SRADio1.encode_and_transmit(tx_message, tx_message_length);
                pktTimer = millis();
                Serial.println("sent");
            }
            if(millis() > sendTimer + 5000){
                Serial.println("Taking a break...");
                sending = false;
                listenTimer = millis();
            }
        }
        else
        { //listening
            u_int8_t rx = SRADio1.tryToRX(rx_message, tx_message_length);
            if (rx == 1 || rx == 3)
            {
                Serial.println("Heard Packet");
                listenTimer = millis();
            }
            if(millis() > listenTimer + 5000){
                Serial.println("Scilence... Starting Beacon!");
                sending = true;
                sendTimer = millis();
            }
        }
        digitalWrite(4,sending);
        Serial.println("loop");
        delay(100);
    }
}
