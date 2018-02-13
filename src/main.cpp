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

SRADio SRADio1;
Hermes Skybass(Serial1);
skybass_data_t sb_data;
bool new_sb_data;
uint32_t watchdog_timer;
uint32_t last_pkt_num;
radio_packet_t packet;

int main()
{
    Serial.begin(115200);
    Serial.println("SRADio Jan 2018");
    SRADio1.configureRF();
    delay(1000);

    while (true)
    {
        if (Skybass.receiveSkybassData(sb_data))
        {
            watchdog_timer = millis();
            packet.packet_num = trimBits(sb_data.packet_num, 18);
            packet.altitude = compressFloat(sb_data.altitude, -2000.0, 40000.0, 15);
            packet.sb_state = trimBits(sb_data.state, 4);
            packet.battery = compressFloat(sb_data.batt_voltage, 3.0, 4.0, 4);
            packet.lat = compressFloat(sb_data.latitude, 0.0, 10.0, 18);
            packet.lon = compressFloat(sb_data.longitude, 0.0, 10.0, 18);
            packet.gps_lock = sb_data.gps_locked;
            packet.strato_alt = compressFloat(0, -2000.0, 40000.0, 15); //TODO
        }
        if (sb_data.packet_num > last_pkt_num)
        {
            last_pkt_num = sb_data.packet_num;
            SRADio1.encode_and_transmit(&packet, sizeof(packet));
        }
    }
}
