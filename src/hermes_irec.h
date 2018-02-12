#include <Arduino.h>
#include "Hermes.h"

enum PacketType
{
  SKYBASS_DATA,
};

typedef struct __attribute__((__packed__)) skybass_data_t : public hermes_data_t
{
  virtual size_t getSize() { return sizeof(skybass_data_t); };
  uint32_t time;       //number of main loop cycles, about 24 hours @ 3hz
  double altitude;     //meters
  uint8_t status;      //4 bits, of skybass state and error
  double batt_voltage; //in volts
  double latitude;
  double longitude;
  bool gps_locked;
} skybass_data_t;