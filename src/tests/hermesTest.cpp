/* 
#include <Arduino.h>
#include "Hermes.h"
#include "hermes_irec.h"

Hermes Hermes1(Serial1);
bool led = HIGH;

int NOTmain()
{
    Serial.begin(115200);
    pinMode(13, OUTPUT);
    while (true)
    {
        skybass_data_t lol;
        lol.packet_num = millis();
        lol.state = 1;
        lol.latitude = 420.0;
        lol.gps_locked = false;
        Hermes1.sendSkybassData(lol);
        delay(1000);
        digitalWrite(13, led);
        led = !led;
        skybass_data_t rx;
        Serial.println(rx.packet_num);
        if (Hermes1.receiveSkybassData(rx))
        {
            Serial.println(rx.packet_num);
        }
        else
        {
            Serial.println("rip");
        }
    }
}
*/