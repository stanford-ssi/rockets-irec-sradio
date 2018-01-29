#include <Arduino.h>
#include "Hermes.h"


Hermes::Hermes(HardwareSerial serial){
    serialPort = serial;
    serialPort.begin(Hermes::baud);
}

void Hermes::send(hermes_data_t &data_struct){
    union hermes_packet_t{
        hermes_data_t data;
        uint8_t encoded[sizeof(data_struct)];
    } packet;
    
    packet.data = data_struct;
    
    serialPort.write(packet.encoded, sizeof(packet.encoded));
}

void Hermes::receive(hermes_data_t &data_struct){
    union hermes_packet_t{
        hermes_data_t data;
        uint8_t encoded[sizeof(data_struct)];
    } packet;

    for (uint8_t i = 0; i < sizeof(packet.encoded); i++)
          packet.encoded[i] = serialPort.read();

    data_struct = packet.data;
}

