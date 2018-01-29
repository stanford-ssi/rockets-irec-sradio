#ifndef _HERMES_H
#define _HERMES_H

#include <Arduino.h>
//This file, which is shared between Skybass and SRADIO, defines the protocol by which data is transmitted between them.
//its called hermes?

typedef struct __attribute__((__packed__)) hermes_data_t {
  
} hermes_data_t;

/* Generated with a fair dice. */
//uint8_t RADIO_START_SEQUENCE[] = {204, 105, 119, 82};
//uint8_t RADIO_END_SEQUENCE[] = {162, 98, 128, 161};

class Hermes{
  public:
    Hermes(HardwareSerial serial);
    void send(hermes_data_t &data_struct);
    void receive(hermes_data_t &data_struct);
  private:  
    HardwareSerial serialPort;
    const uint32_t baud = 115200;
};

#endif


