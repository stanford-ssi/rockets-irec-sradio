#include <Arduino.h>
#include <ArduinoJson.h>
#include "SRADio.h"
#include "utils.h"
#include "irec-rf.h"

int main()
{
    Serial.begin(115200);
    Serial.println("SRADio Feb 2018");
    SRADio SRADio1;
    SRADio1.configureRF();
    rf_down_t down_pkt;
    rf_up_t up_pkt;
    up_pkt.charge1 = 0;
    up_pkt.charge2 = 0;
    up_pkt.charge3 = 0;
    up_pkt.charge4 = 0;
    up_pkt.charge1_parity = 0;
    up_pkt.charge2_parity = 0;
    up_pkt.charge3_parity = 0;
    up_pkt.charge4_parity = 0;
    up_pkt.arm_payload = 0;
    bool boom = false;
    bool payload = true;

    while (true)
    {
        uint8_t rx = SRADio1.tryToRX(&down_pkt, sizeof(down_pkt));
        if (rx == 1 || rx == 3)
        {

            StaticJsonBuffer<2000> jsonBuffer;
            JsonObject &root = jsonBuffer.createObject();
            root["packet_num"] = down_pkt.packet_num;
            root["altitude"] = expandFloat(down_pkt.altitude, -2000.0, 40000.0, 15);
            root["sb_state"] = down_pkt.sb_state;
            root["battery"] = expandFloat(down_pkt.battery, 3.0, 4.0, 8);
            root["lat"] = expandFloat(down_pkt.lat, 0.0, 10.0, 18);
            root["lon"] = expandFloat(down_pkt.lon, 0.0, 10.0, 18);
            root["gps_lock"] = down_pkt.gps_lock;
            root["strato_alt"] = expandFloat(down_pkt.strato_alt, -2000.0, 40000.0, 15);
            root["vsense1"] = unpackBits(down_pkt.vsense1, 8, 0.0, 9.9);
            root["vsense2"] = unpackBits(down_pkt.vsense2, 8, 0.0, 9.9);
            root["payload_armed"] = down_pkt.payload_armed;
            root["skybass_alive"] = down_pkt.skybass_alive;
            root["skybass_armed"] = down_pkt.skybass_armed;
            root["charges_blown"] = down_pkt.charges_blown;
            root["boom_cmd"] = boom;
            root["payload_cmd"] = payload;

            root["rssi"] = SRADio1.getRSSI();
            root["rx_code"] = rx;

            if (bitRead(rx, 1))
            {
                root["syndrome"] = SRADio1.getSyndrome();
            }

            root.printTo(Serial);
            Serial.println();
        }

        if (Serial.available() > 2)
        {
            String word = Serial.readStringUntil('\n');
            if (word.indexOf("BOOM") != -1)
            {
                Serial.println("FUCK WE GOING BOOM");
                up_pkt.charge1 = 1;
                up_pkt.charge2 = 1;
                up_pkt.charge3 = 1;
                up_pkt.charge4 = 1;
                up_pkt.charge1_parity = 1;
                up_pkt.charge2_parity = 1;
                up_pkt.charge3_parity = 1;
                up_pkt.charge4_parity = 1;
                boom = true;
            }

            if (word.indexOf("PAYLOAD") != -1)
            {
                Serial.println("time to arm payload");
                up_pkt.arm_payload = 1;
                payload = true;
            }
            if (word.indexOf("DISARM") != -1)
            {
                Serial.println("Disarm Payload");
                up_pkt.arm_payload = 0;
                payload = false;
            }
            SRADio1.encode_and_transmit(&up_pkt, sizeof(up_pkt));
        }
    }
}
