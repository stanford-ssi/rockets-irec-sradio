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
#include "min-irec.h"
#include "min_support.h"
#include "utils.h"
#include "irec-rf.h"
#include "deadman.cpp"

#include "min.h"
#include "min.c"


struct min_context min_ctx;
esp_status_t esp_status;
skyb_data_t sb_data;
bool new_esp_status = false;
bool new_sb_data = false;

void min_application_handler(uint8_t min_id, uint8_t *min_payload, uint8_t len_payload, uint8_t port)
{
    switch (min_id)
    {
    case SKYB_DATA:
        if (len_payload == sizeof(skyb_data_t))
        {
            memcpy(&sb_data, min_payload, len_payload);
            new_sb_data = true;
        }
        else
        {
            Serial.printf("Size Err: %i/%i", len_payload, sizeof(skyb_data_t));
        }
        break;

    case ESP_STATUS:
        if (len_payload == sizeof(esp_status_t))
        {
            memcpy(&esp_status, min_payload, len_payload);
            new_esp_status = true;
        }
        else
        {
            Serial.printf("Size Err: %i/%i", len_payload, sizeof(esp_status_t));
        }
        break;
    }
}

int main()
{
    pinMode(WATCHDOG_PIN, OUTPUT);
    Serial.begin(115200);
    Serial.println("SRADio Apr 2018");

    min_init_context(&min_ctx, 1);

    uint32_t tx_timer = 0;

    radio_packet_t packet;

    SRADio SRADio1;
    DeadMan beacon(WATCHDOG_PIN,3000L);
    SRADio1.configureRF();

    Serial.println("1");
    delay(3000);

    while (true)
    {
        Serial.print(".");

        //MIN Serial polling code
        char buf[32];
        size_t buf_len;
        if(Serial1.available() > 0) {
            buf_len = Serial1.readBytes(buf, 32U);
        }else {
            buf_len = 0;
        }
        min_poll(&min_ctx, (uint8_t *)buf, (uint8_t)buf_len);
        

        //compress downlink data from skybass
        if (new_sb_data)
        {
            beacon.pet();
            packet.packet_num = trimBits(sb_data.packet_num, 18);
            packet.altitude = compressFloat(sb_data.altitude, -2000.0, 40000.0, 15);
            packet.sb_state = trimBits(sb_data.state, 4);
            packet.battery = compressFloat(sb_data.batt_voltage, 3.0, 4.0, 8);
            packet.lat = compressFloat(sb_data.latitude, 0.0, 10.0, 18);
            packet.lon = compressFloat(sb_data.longitude, 0.0, 10.0, 18);
            packet.gps_lock = sb_data.gps_locked;
            new_sb_data = false;
        }

        //compress downlink data from ESPs
        if (new_esp_status){
            packet.skybass_alive = esp_status.skybass_alive;
            packet.skybass_armed = esp_status.skybass_armed;
        }

        //compress downlink data from voltage sense
        uint32_t voltage_1 = analogRead(VS1);
        uint32_t voltage_2 = analogRead(VS2);
        packet.vsense1 = fitToBits(voltage_1, 8, 0, 1023);
        packet.vsense2 = fitToBits(voltage_2, 8, 0, 1023);
        packet.strato_alt = compressFloat(0, -2000.0, 40000.0, 15); //TODO

        //send telemetry packet
        if (millis() - tx_timer > 50)
        {
            Serial.println("Sent Radio Packet");
            tx_timer = millis();
            SRADio1.encode_and_transmit(&packet, sizeof(packet)); //SEND
        }


        beacon.check();
    }
}

void min_tx_byte(uint8_t port, uint8_t byte)
{
    switch (port)
    {
    case 1:
        Serial1.write(&byte, 1U);
        break;
    }
}

uint16_t min_tx_space(uint8_t port)
{
  switch (port)
    {
    case 1:
        return Serial1.availableForWrite();
        break;
    }
    return 0;
}
