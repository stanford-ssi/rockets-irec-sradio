#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>


double_t map_float(double_t x, double_t in_min, double_t in_max, double_t out_min, double_t out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint32_t compressFloat(double_t value, double_t min, double_t max, uint8_t bits)
{   
    uint32_t range = pow(2,bits) - 1;
    //mod to the input bounds provided
    while (value > max)
        value = value - max + min;
    while (value < min)
        value = value + max - min;
    //map to whole number increments
    value = map(value, min, max, 0.0, (double)range);
    //constrain to bit range and reduce to int
    uint32_t quantized = constrain((uint32_t)value, 0, range);
    //mask to the bit range
    quantized = quantized & range;
    return quantized;
}

uint32_t trimBits(uint32_t value, uint8_t bits)
{
    uint32_t max = pow(2,bits) - 1;
    //mod to the input bounds provided
    value = value % max;
    //constrain to bit range and reduce to int
    constrain(value, 0, max);
    //mask to the bit range
    value = value & max;
    return value;
}

uint32_t fitToBits(uint32_t value, uint8_t bits, uint32_t min, uint32_t max){

    uint32_t out_max = pow(2,bits) - 1;
    //mod to the input bounds provided
    while (value > max)
        value = value - max + min;
    while (value < min)
        value = value + max - min;
    //constrain to bit range and reduce to int
    value = map(value,min,max,0,out_max);

    return trimBits(value,bits);
}

double_t unpackBits(uint32_t value, uint8_t bits, double_t min, double_t max){

    uint32_t in_max = pow(2,bits) - 1;
    double_t val = map_float(value,0,in_max,min,max);
    return val;
}

double_t expandFloat(uint32_t value, double_t min, double_t max, uint8_t bits){
    uint32_t range = pow(2,bits) - 1;
    double_t scaled = map((double_t)value, 0.0, (double_t)range, min, max);
    return scaled;
}


#endif