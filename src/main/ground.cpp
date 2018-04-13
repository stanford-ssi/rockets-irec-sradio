#include <Arduino.h>
#include <ArduinoJson.h>
#include "SRADio.h"
#include "utils.h"

int main()
{
    Serial.begin(115200);
    Serial.println("SRADio Feb 2018");
    SRADio SRADio1;
    SRADio1.configureRF();
    radio_packet_t packet; 

    while (true)
    {
        uint8_t rx = SRADio1.tryToRX(&packet, sizeof(packet));
        if(rx == 1 || rx == 3){

        StaticJsonBuffer<2000> jsonBuffer;   
        JsonObject& root = jsonBuffer.createObject();
        root["packet_num"] = packet.packet_num;
        root["altitude"] = expandFloat(packet.altitude, -2000.0, 40000.0, 15);
        root["sb_state"] = packet.sb_state;
        root["battery"] = expandFloat(packet.battery, 3.0, 4.0, 8);
        root["lat"] = expandFloat(packet.lat, 0.0, 10.0, 18);
        root["lon"] = expandFloat(packet.lon, 0.0, 10.0, 18);
        root["gps_lock"] = packet.gps_lock;
        root["strato_alt"] = expandFloat(packet.strato_alt, -2000.0, 40000.0, 15);
        root["vsense1"] = unpackBits(packet.vsense1,8,0.0,9.9);
        root["vsense2"] = unpackBits(packet.vsense2,8,0.0,9.9);
        root["rssi"] = SRADio1.getRSSI();
        root["rx_code"] = rx;
        
        if(bitRead(rx,1)){
            root["syndrome"] = SRADio1.getSyndrome();
        }
        
        root.printTo(Serial);
        Serial.println();
        }
    }
}
