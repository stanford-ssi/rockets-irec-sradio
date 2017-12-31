#include <Arduino.h>
#include "config.h"
#include "RH_RF24.h"
#include "ecc.h"

/*
SRADio GFSK Library
This library provides an abstraction for the SRADio hardware developed by SSI.
*/

class SRADio
{
public:
  SRADio();
  void configureRF();
  void encode_and_transmit(uint8_t *latest_frame, uint8_t frame_size);
  uint8_t tryToRX(uint8_t *message);
  uint8_t getRSSI();

private:
  void RadioOff();
  void RadioOn();
  void configurePins();
  RH_RF24 *rf24; //the RadioHead Driver Object
  uint8_t lastRssi; //RSSI of last reception
};
