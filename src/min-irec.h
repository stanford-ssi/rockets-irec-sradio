#include <Arduino.h>

//this file defines the packet structs for IREC 2018. There are unused feilds. See Google Sheet for detials on each packet.
//@timv 2018

enum PacketType
{
  ESP_ARM,
  ESP_STAGE,
  ESP_STATUS,
  SKYB_DATA,
  SKYB_CMD,
};

typedef struct __attribute__((__packed__)) esp_arm_t
{
  bool arm_payload;
  bool arm_strato;
  bool arm_skybass;
} esp_arm_t;

typedef struct __attribute__((__packed__)) esp_stage_t
{
  bool do_stage;
} esp_stage_t;

typedef struct __attribute__((__packed__)) esp_status_t
{
  bool payload_alive;
  bool staging_alive;
  bool cots_alive;
  bool skybass_alive;
  bool payload_armed;
  bool cots_armed;
  bool skybass_armed;
  bool staging_activated;
  bool light_sensor;
} esp_status_t;

typedef struct __attribute__((__packed__)) skyb_data_t
{
  uint32_t packet_num; //number of main loop cycles, about 24 hours @ 3hz
  double altitude;     //meters
  uint8_t state;       //4 bits, of skybass state and error
  double batt_voltage; //in volts
  double latitude;
  double longitude;
  bool gps_locked;
} skyb_data_t;

typedef struct __attribute__((__packed__)) skyb_cmd_t
{
  bool reset;
  bool ematch1;
  bool ematch2;
  bool ematch3;
  bool ematch4;
} skyb_cmd_t;