#include <Arduino.h>
#include "Hermes.h"

//something with enums and message types here!

typedef struct __attribute__((__packed__)) skybass_data_t : hermes_data_t {
  uint32_t time; //number of main loop cycles, about 24 hours @ 3hz
  double altitude; //meters
  uint8_t status; //4 bits, of skybass state and error
  double batt_voltage; //in volts
  double latitude;
  double longitude;
  bool gps_locked;
} skybass_data_t;