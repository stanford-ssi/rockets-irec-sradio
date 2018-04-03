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
#include "utils.h"



int main()
{
    pinMode(WATCHDOG_PIN, OUTPUT);
    Serial.begin(115200);
    Serial.println("SRADio Apr 2018");

    skybass_data_t sb_data;
    bool new_sb_data;
    uint32_t watchdog_timer =0 ;
    uint32_t pulse_timer = 0;
    uint32_t tx_timer = 0;
    bool pulse_state = false;
    radio_packet_t packet;

    SRADio SRADio1;
    Hermes Skybass(SKYBASS_SERIAL);
    SRADio1.configureRF();

    Serial.println("1");
    delay(5000);
    Serial.println(sb_data.getSize());

    while (true)
    {
        Serial.print(".");
        if(Skybass.receiveSkybassData(sb_data))
        {
            watchdog_timer = millis();
            Serial.println("Got Skybass Packet");

            uint8_t voltage_1 = analogRead(VS1)/8;
            uint8_t voltage_2 = analogRead(VS2)/8;
            packet.vsense1 = trimBits(voltage_1, 4);
            packet.vsense2 = trimBits(voltage_2, 4);

            packet.packet_num = trimBits(sb_data.packet_num, 18);
            packet.altitude = compressFloat(sb_data.altitude, -2000.0, 40000.0, 15);
            packet.sb_state = trimBits(sb_data.state, 4);
            packet.battery = compressFloat(sb_data.batt_voltage, 3.0, 4.0, 4);
            packet.lat = compressFloat(sb_data.latitude, 0.0, 10.0, 18);
            packet.lon = compressFloat(sb_data.longitude, 0.0, 10.0, 18);
            packet.gps_lock = sb_data.gps_locked;
            packet.strato_alt = compressFloat(0, -2000.0, 40000.0, 15); //TODO
            
        }

        if(millis() - tx_timer > 50){
            Serial.println("Sent Radio Packet");
            tx_timer = millis();
            SRADio1.encode_and_transmit(&packet, sizeof(packet));//SEND
        }
        
        if (millis() > (watchdog_timer + 3000))
        { //if skybass is still alive
            if (millis() > pulse_timer + 200)
            { //if its been a while...
                pulse_state = !pulse_state;
                digitalWrite(WATCHDOG_PIN, pulse_state);
            }
        }

    }
}
