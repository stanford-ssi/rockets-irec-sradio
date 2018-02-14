#include <Arduino.h>
#include <ArduinoJson.h>
#include "SRADio.h"
#include "utils.h"

SRADio SRADio1;
radio_packet_t packet;
StaticJsonBuffer<200> jsonBuffer;

int main()
{
    Serial.begin(115200);
    Serial.println("SRADio Feb 2018");
    SRADio1.configureRF();

    while (true)
    {
        uint8_t rx = SRADio1.tryToRX(&packet, sizeof(packet));
        if(rx == 1 || rx == 3){
            
        JsonObject& root = jsonBuffer.createObject();
        root["packet_num"] = packet.packet_num;
        root["altitude"] = expandFloat(packet.altitude, -2000.0, 40000.0, 15);
        root["sb_state"] = packet.sb_state;
        root["battery"] = expandFloat(packet.battery, 3.0, 4.0, 4);
        root["lat"] = expandFloat(packet.lat, 0.0, 10.0, 18);
        root["lon"] = expandFloat(packet.lon, 0.0, 10.0, 18);
        root["gps_lock"] = packet.gps_lock;
        root["strato_lock"] = expandFloat(packet.strato_alt, -2000.0, 40000.0, 15);
        root["rssi"] = SRADio1.getRSSI();
        
        root.printTo(Serial);
        Serial.println();
        }
    }
}