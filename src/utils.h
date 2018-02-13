#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

typedef struct __attribute__((__packed__)) radio_packet_t
{ //should be 12 bytes
    unsigned packet_num : 18;
    unsigned altitude : 15;
    unsigned sb_state : 4;
    unsigned battery : 4;
    unsigned lat : 18;
    unsigned lon : 18;
    unsigned gps_lock : 1;
    unsigned strato_alt : 15;
    unsigned padding : 3;
} radio_packet_t;

uint32_t compressFloat(double_t value, double_t min, double_t max, uint8_t bits)
{
    //mod to the input bounds provided
    while (value > max)
        value = value - max + min;
    while (value < min)
        value = value + max - min;
    //map to whole number increments
    value = map(value, min, max, 0.0, ((double)(2 ^ bits) - 1));
    //constrain to bit range and reduce to int
    uint32_t quantized = constrain((uint32_t)value, 0, (2 ^ bits) - 1);
    //mask to the bit range
    quantized = quantized & ((uint32_t)(2 ^ bits) - 1);
    return quantized;
}

uint32_t trimBits(uint32_t value, uint8_t bits)
{
    uint32_t max = (2 ^ bits) - 1;
    //mod to the input bounds provided
    value = value % max;
    //constrain to bit range and reduce to int
    constrain(value, 0, max);
    //mask to the bit range
    value = value & max;
    return value;
}

#endif