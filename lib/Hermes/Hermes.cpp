#include <Arduino.h>
#include "Hermes.h"
//the right way to do this is to register callbacks for the message handler, but it dosen't matter and for not im just gonna jank.

uint8_t RADIO_START_SEQUENCE[] = {204, 105, 119, 82};
uint8_t RADIO_END_SEQUENCE[] = {162, 98, 128, 161};

Hermes::Hermes(HardwareSerial serial)
{
    serialPort = serial;
    Serial1.begin(115200);
}

void Hermes::send(hermes_data_t &data_struct)
{
    uint8_t data[data_struct.getSize()];
    memcpy(data, &data_struct, data_struct.getSize());
    Serial1.write(data, data_struct.getSize());
}

void Hermes::sendSkybassData(hermes_data_t &data_struct)
{
    Serial1.write(RADIO_START_SEQUENCE, 4);
    send(data_struct);
    Serial1.write(RADIO_END_SEQUENCE, 4);
}

bool Hermes::receiveSkybassData(hermes_data_t &data_struct)
{
    uint8_t last_bytes[4] = {0, 0, 0, 0};
    delay(10);
    while (Serial1.available() > 0)
    {
        uint8_t byte = Serial1.read();
        last_bytes[0] = last_bytes[1];
        last_bytes[1] = last_bytes[2];
        last_bytes[2] = last_bytes[3];
        last_bytes[3] = byte;
        if (*(uint32_t *)RADIO_START_SEQUENCE == *(uint32_t *)last_bytes)
        {
            parse(data_struct);
            last_bytes[0] = Serial1.read();
            last_bytes[1] = Serial1.read();
            last_bytes[2] = Serial1.read();
            last_bytes[3] = Serial1.read();
            if (*(uint32_t *)RADIO_END_SEQUENCE != *(uint32_t *)last_bytes)
            {
                Serial.println("Skybass Packet Receive Error");
                return false;
            }
            else
            {
                Serial.println("Received Skybass Packet");
                return true;
            }
        }
    }
    return false;
}

void Hermes::parse(hermes_data_t &data_struct)
{
    Serial.println("Start");
    uint8_t size = 42;
    //uint8_t size = data_struct.getSize();//This is the fucked shit that you just have to use "42" or it crashes cuz C++ is bad
    uint8_t data[size];
    Serial.println(size);
    for (uint8_t i = 0; i < size; i++){
        data[i] = Serial1.read();
    }
    memcpy(&data_struct, &data, size);

}