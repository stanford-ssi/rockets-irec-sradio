#ifndef MIN_SUPPORT_H
#define MIN_SUPPORT_H

#include <Arduino.h>
#include "min.h"

//this file adapts the MIN C implementation to the Arduino Framework. You should adapt the switch-cases
//for your port configuration. for each port you will need a min context, initialized to the right number.

//ask @timv if you have questions.
//This should eventually be rolled into a min C++ arduino implementation.

void min_tx_byte(uint8_t port, uint8_t byte)
{
    switch (port)
    {
    case 0:
        Serial1.write(&byte, 1U);
        break;
        //case 1:
        //break;
    }
}

uint32_t min_time_ms(void)
{
    return millis();
}

uint16_t min_tx_space(uint8_t port)
{
  switch (port)
    {
    case 0:
        return Serial1.availableForWrite();
        break;
    //case 1:
        //break;
    }
    return 0;
}

void min_tx_start(uint8_t port){
//nada
}

void min_tx_finished(uint8_t port){
//nada
}

#endif