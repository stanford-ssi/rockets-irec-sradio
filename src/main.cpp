/*
 Project: ValBal EE 8.1 Radio Code
 Created: 08/26/2017 12:33:25 PM
 Authors:  Aria Tedjarati
           Joan Creus-Costa

 Description:
 APRS, SSTV, GFSK (2-way) radio communication as a payload board for ValBal EE 8.1 system

 Credits:
 Femtoloon and Iskender Kushan's SSTV code.

 Status:
 07/09/2017 Starting
 08/26/2017 Hardware confirmed working, SSTV, .wav file works, GFSK works, power works.
 08/27/2017 APRS works.  All hardware is complete. The board is fully functional.
 09/17/2017 Test code for the range test is written.
 09/20/2017 Flight-ish code

 TODO as of 09/20/2017:
 Logging. More testing.
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RH_RF24.h>
#include <Audio.h>
#include <SerialFlash.h>
#include <aprs.h>
#include "RadioInterface.h"

const int MESSAGE_LENGTH = 50;
#define NPAR 40

#include "ecc.h"

#define   PAYLOAD_1          4
#define   SCAP_PG            27
#define   EN_5V              30
#define   GFSK_SDN           6
#define   GFSK_IRQ           2
#define   GFSK_GATE          5
#define   GFSK_GPIO_0        21
#define   GFSK_GPIO_1        20
#define   GFSK_GPIO_2        7
#define   GFSK_GPIO_3        8
#define   GFSK_CS            25
#define   BOOST_EN           16
#define   SD_CS              23
#define   PAYLOAD_4          3
#define   DORJI_SLP          15
#define   DORJI_SQ           28
#define   DORJI_GATE         31        // Dorji on/off switch.  Write high for off, low for on.
#define   DORJI_PSEL         32        // Dorji power select pin. Write high for high power, low for low power.
#define   DORJI_AUD          A0        // Dorji audio output to MCU.
#define   DORJI_PTT          17
#define   V_SPRCAP           A10       // Supercapacitor voltage analog input.
#define   BAT_V              A11       // Battery voltage analog input.
#define   DORJI_DAC          A14       // DAC output to Dorji
#define   UART_DORJI         Serial1
#define   UART_MAINBOARD     Serial2

File                     logfile;                    // SD Card Logfile
AudioPlaySdWav           playWav1;                   // SSTV Audio File
AudioOutputAnalog        audioOutput;
AudioConnection          patchCord1(playWav1, 0, audioOutput, 0);
AudioConnection          patchCord2(playWav1, 1, audioOutput, 1);
AudioControlSGTL5000     sgtl5000_1;

// Constants and Tuneable Variables
const int     DORJI_BAUD =             9600;
const int     DRA818V_PTT_DELAY =      150;             // Milliseconds to wait after PTT to transmit
const int     DRA818V_COMMAND_DELAY =  250;             // Milliseconds to wait after sending a UART command
const int     DORJI_WAKEUP_INT =       5000;            // Milliseconds to wake dorji up
#define       S_CALLSIGN               "KK6MIS"         // FCC Callsign
#define       S_CALLSIGN_ID            1                // 11 is usually for balloons
#define       D_CALLSIGN               "APRS"           // Destination callsign: APRS (with SSID=0) is usually okay.
#define       D_CALLSIGN_ID            0                // Desination ID
#define       SYMBOL_TABLE             '/'              // Symbol Table: '/' is primary table '\' is secondary table
#define       SYMBOL_CHAR              'v'              // Primary Table Symbols: /O=balloon, /-=House, /v=Blue Van, />=Red Car

struct PathAddress addresses[] = {
  { (char *)D_CALLSIGN, D_CALLSIGN_ID },  // Destination callsign
  { (char *)S_CALLSIGN, S_CALLSIGN_ID },  // Source callsign
  { (char *)NULL, 0 }, // Digi1 (first digi in the chain)
  { (char *)NULL, 0 }  // Digi2 (second digi in the chain)
};

RH_RF24                  rf24(GFSK_CS, GFSK_IRQ, GFSK_SDN);

typedef enum {
        DO_THE_THINGS,
        GFSK_CONFIG,
        TRANSMIT_GFSK,
        SSTV_CHARGING,
        SSTV_CONFIG,
        TRANSMIT_SSTV,
        APRS_CONFIG,
        APRS_CHARGING,
        TRANSMIT_APRS
} states;

states        currentState = GFSK_CONFIG;
states        previousState;
states        nextState;

const int AUTOMATIC = 1;
const int MANUAL = 2;

int RADIO_MODE = AUTOMATIC;

vb_rf_message message;
bool parsing = true;
uint8_t parse_pos = 0;
uint8_t last_bytes[4] = {0};
uint8_t frame_size;
uint8_t latest_frame[256];

const char* callsign_message = "This is ValBal, callsign KK6MIS, a Stanford balloon payload, visit habmc.stanfordssi.org. ";

float yeLatitude = 0.0;
float yeLongitude = 0.0;

float goodLatitude = 37.427501;
float goodLongitude = -122.170269;

#include "BoardUtils.h"

double configFrequency = 433.5;
float configAPRSInterval = 10*60*1000;
int configConfig = 5;
int configCallsignDivider = 100;

int sendRadioTimer = millis() + 3000;
int sendAPRSTimer = sendRadioTimer + 60*1000;

void parse_radio_config(vb_rf_config* config) {
  switch (config->config) {
  case VB_FREQUENCY:
    Serial.print("   - Setting ValBal frequency to ");
    configFrequency = config->data[0] + ((float) config->data[1])/65536.;
    Serial.println(configFrequency);
    nextState = GFSK_CONFIG;
    break;
  case BEC_INTERVAL:
    Serial.print("   - Setting BEC interval to ");
    Serial.println(config->data[0]);
    break;
  case APRS_INTERVAL:
    Serial.print("   - Setting APRS interval to ");
    Serial.println(config->data[0]);
    Serial.println("minutes");
    configAPRSInterval = config->data[0] * 60000;
    sendAPRSTimer = millis() + configAPRSInterval;
    break;
  case VB_DATARATE:
    Serial.print("   - Setting datarate to ");
    Serial.println(config->data[0]);
    configConfig = config->data[0];
    if (configConfig > 10 || configConfig < 1) configConfig = 5;
    nextState = GFSK_CONFIG;
    break;
  case RF_MODE:
    Serial.print("   - Setting RF mode to ");
    Serial.println(config->data[0]);
    if (config->data[0] == 1776) {
      RADIO_MODE = AUTOMATIC;
      nextState = GFSK_CONFIG;
      sendRadioTimer = millis() + 6000;
      sendAPRSTimer = sendRadioTimer + configAPRSInterval;
    } else {
      RADIO_MODE = MANUAL;
      GFSKOff();
    }
    break;
  case APRS_NOW:
    Serial.println("   - Sending APRS now whaddup ");
    nextState = APRS_CONFIG;
    break;
  case CALLSIGN_DIVIDER:
    configCallsignDivider = config->data[0];
    if (configCallsignDivider == 0) configCallsignDivider = 100;
  }
}

void parse_radio_command() {
  //Serial.println("Parsing radio command.");
  switch (message.type) {

  case HEARTBEAT:
    //Serial.print(" > Answering to heartbeat ");
    //Serial.println(message.data[0], DEC);
    digitalWrite(PAYLOAD_4, message.data[0] == 0 ? LOW : HIGH);
    break;

  case SET_CONFIG:
    Serial.print(" > Getting config, ");
    Serial.print(message.data[0]);
    Serial.println(" changes.");
    for (int i=0; i<message.data[0]; i++) {
      parse_radio_config((vb_rf_config*)(message.data + 1 + sizeof(vb_rf_config)*i));
    }
    break;

  case DATA_FRAME:
    Serial.print(" >");
    int sz = message.data[0];
    yeLatitude = *(float*)(message.data+1);
    yeLongitude = *(float*)(message.data+5);
    if (fabs(yeLatitude) > 0.01 && fabs(yeLongitude) > 0.01) {
      goodLatitude = yeLatitude;
      goodLongitude = yeLongitude;
    }
    Serial.print("Got ");
    Serial.print(yeLatitude);
    Serial.print(" ");
    Serial.println(yeLongitude);
    memcpy(latest_frame, message.data+9, sz);
    frame_size = sz;
    break;
  }

}


void receive_byte() {
  if (Serial2.available() > 0) {
    uint8_t byte = Serial2.read();
    last_bytes[0] = last_bytes[1];
    last_bytes[1] = last_bytes[2];
    last_bytes[2] = last_bytes[3];
    last_bytes[3] = byte;
    if (*(uint32_t*)RADIO_START_SEQUENCE == *(uint32_t*)last_bytes) {
      parsing = true;
      parse_pos = 0;
      memset(&message, 0, sizeof(vb_rf_message));
      return;
    }
    if (*(uint32_t*)RADIO_END_SEQUENCE == *(uint32_t*)last_bytes) {
      parsing = false;
      ((uint8_t*)&message)[parse_pos] = 0;
      ((uint8_t*)&message)[parse_pos-1] = 0;
      ((uint8_t*)&message)[parse_pos-2] = 0;
      ((uint8_t*)&message)[parse_pos-3] = 0;
      parse_radio_command();
    }
    if (parsing && parse_pos < sizeof(vb_rf_message)) {
      ((uint8_t*)&message)[parse_pos] = byte;
      parse_pos++;
    }
  }
}

void playFile(const char *filenamew) {
  Serial.print("Playing SSTV file: ");
  Serial.println(filenamew);
  Serial.println(playWav1.play(filenamew));
  delay(5);
  while (playWav1.isPlaying()) {
  }
}

int main() {
  pinMode(BOOST_EN, OUTPUT);
  pinMode(EN_5V, OUTPUT);
  pinMode(DORJI_GATE, OUTPUT);
  pinMode(GFSK_GATE, OUTPUT);
  pinMode(DORJI_SLP, OUTPUT);
  pinMode(DORJI_PTT, OUTPUT);
  pinMode(DORJI_PSEL, OUTPUT);
  pinMode(GFSK_SDN,OUTPUT);
  amBusy();
  SupercapChargerOff();
  FiveVOff();
  DorjiOff();
  GFSKOff();
  DorjiSleep();
  DorjiPTTOff();
  DorjiLowPower();
  Wire.begin();//I2C_MASsTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
  lowBoostPower();
  //highBoostPower();
  SPI.setSCK(13);       // SCK on pin 13
  SPI.setMOSI(11);      // MOSI on pin 11
  SPI.setMISO(12);      // MOSI on pin 11
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV2);  // Setting clock speed to 8mhz, as 10 is the max for the rfm22
  SPI.begin();
  Serial.begin(115200);
  Serial.println("ValBal 8.1 Radio Board, Summer 2017");
  analogReadResolution(12);
  analogReference(INTERNAL);
  //setupSDCard();
  AudioMemory(8);
  UART_DORJI.begin(DORJI_BAUD);
  aprs_setup(50, // number of preamble flags to send
    DORJI_PTT, // Use PTT pin
    500, // ms to wait after PTT to transmit
    0, 0 // No VOX ton
  );

  pinMode(PAYLOAD_1, OUTPUT);
  pinMode(PAYLOAD_4, OUTPUT);

  initialize_ecc();

  delay(3000);
  Serial2.begin(VBRF_BAUD_RATE);


  uint8_t data[256] = {0};
  uint8_t leng = MESSAGE_LENGTH + NPAR;
  int nAddresses = 0;
  int msgNum = 0;

  sendRadioTimer = millis() + 3000;
  sendAPRSTimer = sendRadioTimer + 60*1000;

  while (true) {
        uint32_t st = micros();
    switch (currentState) {
    case DO_THE_THINGS:
      if (RADIO_MODE == MANUAL || (RADIO_MODE == AUTOMATIC)) amFree();
      else amBusy();
      receive_byte();
      if (nextState != DO_THE_THINGS) {
        parsing = false;
        parse_pos = 0;
        break;
      }
      if (RADIO_MODE == AUTOMATIC) {
        if (millis() > sendAPRSTimer) {
          nextState = APRS_CHARGING;
        } else if (millis() > sendRadioTimer) {
          nextState = TRANSMIT_GFSK;
        }
      } else {
        nextState = DO_THE_THINGS;
      }
      break;

    case GFSK_CONFIG:
      amBusy();
      GFSKOff();
      delay(500);
      GFSKOn();
      rf24.init(MESSAGE_LENGTH+NPAR);
      uint8_t buf[8];
      if (!rf24.command(RH_RF24_CMD_PART_INFO, 0, 0, buf, sizeof(buf))) {
        Serial.println("SPI ERROR");
      } else {
        Serial.println("SPI OK");
      }
      if (!rf24.setFrequency(configFrequency)) {
        Serial.println("setFrequency failed");
      } else {
        Serial.print("Frequency set to ");
        Serial.print(configFrequency);
        Serial.println(" MHz.");
      }
      //rf24.setModemConfig(rf24.GFSK_Rb0_5Fd1);   // GFSK 500 bps

      if (configConfig == 1) rf24.setModemConfig(rf24.FSK_Rb0_5Fd1);    // FSK  500 bps
      if (configConfig == 2) rf24.setModemConfig(rf24.FSK_Rb5Fd10);     // FSK  5   kbps
      if (configConfig == 3) rf24.setModemConfig(rf24.FSK_Rb50Fd100);   // FSK  50  kbps
      if (configConfig == 4) rf24.setModemConfig(rf24.FSK_Rb150Fd300);  // FSK  150 kbps
      if (configConfig == 5) rf24.setModemConfig(rf24.GFSK_Rb0_5Fd1);   // GFSK 500 bps
      if (configConfig == 6) rf24.setModemConfig(rf24.GFSK_Rb5Fd10);    // GFSK 5   kbps
      if (configConfig == 7) rf24.setModemConfig(rf24.GFSK_Rb50Fd100);  // GFSK 50  kbps
      if (configConfig == 8) rf24.setModemConfig(rf24.GFSK_Rb150Fd300); // GFSK 150 kbps
      if (configConfig == 9) rf24.setModemConfig(rf24.OOK_Rb5Bw30);     // OOK  5   kbps
      if (configConfig == 10) rf24.setModemConfig(rf24.OOK_Rb10Bw40);   // OOK  10  kbps

      Serial.println("set up");

      rf24.setTxPower(0x7f);
      nextState = DO_THE_THINGS;
      break;

    case TRANSMIT_GFSK:
      amBusy();

      if (frame_size > MESSAGE_LENGTH) {
        Serial.println("WELL CRAP");
      }

      if ((msgNum % configCallsignDivider == 10)) {
        memcpy(data, callsign_message, 90);
      } else {
        encode_data(latest_frame, frame_size, data);
        Serial.println("encoded data");
        for (int i=0; i<frame_size+NPAR; i++) {
          int k = data[i];
          Serial.print(k);
          if (k < 10) Serial.print(" ");
          if (k < 100) Serial.print(" ");
          Serial.print(" ");
        }
        Serial.println();
      }
      msgNum++;

      rf24.send(data, MESSAGE_LENGTH + NPAR);
      rf24.waitPacketSent();

      Serial.print((micros()-st)/1000.);
      Serial.println(" ms");

      sendRadioTimer = millis() + 2000;

      nextState = DO_THE_THINGS;
      break;

    case SSTV_CHARGING:
      amBusy();
      SupercapChargerOn(); // Turn on sprcap charger
      while(superCapVoltage() <= 5.0) {
        delay(1000);
        Serial.println("Waiting for supercap to charge.  It is currently at: ");
        Serial.print(superCapVoltage()); Serial.println(" volts.");
      }
      FiveVOn();  // Turn on 5V line
      delay(500);
      nextState = SSTV_CONFIG;
      break;

    case SSTV_CONFIG:
      amBusy();
      Serial.println("Configuring Dorji for SSTV transmit");
      DorjiOn();
      delay(1000);
      DorjiSleep();
      delay(1000);
      DorjiWake();
      delay(5000);
      DorjiLowPower();
      delay(1000);
      UART_DORJI.print("AT+DMOCONNECT\r\n"); //handshake command
      delay(1000);
      UART_DORJI.print("AT+DMOSETGROUP=1,144.5000,144.5000,0000,4,0000\r\n"); //set tx and rx frequencies
      delay(1000);
      DorjiSleep();
      delay(3000);
      DorjiWake();
      delay(5000);
      nextState = TRANSMIT_SSTV;
      break;

    case TRANSMIT_SSTV:
      amBusy();
      Serial.println("Transmitting SSTV");
      DorjiPTTOn(); // enable push to talk
      delay(1000);
      playFile("SSTVV.WAV");
      DorjiPTTOff(); // disable push to talk
      DorjiSleep(); //sleep sweet dorji <3
      delay(1000);
      DorjiOff();
      FiveVOff();
      nextState = DO_THE_THINGS;
      break;

    case APRS_CHARGING:
      amBusy();
      SupercapChargerOn(); // Turn on sprcap charger
      while(superCapVoltage() <= 5.0) {
        delay(1000);
        Serial.println("Waiting for supercap to charge.  It is currently at: ");
        Serial.print(superCapVoltage()); Serial.println(" volts.");
      }
      FiveVOn();  // Turn on 5V line
      delay(500);
      nextState = APRS_CONFIG;
      break;

    case APRS_CONFIG:
      amBusy();
      Serial.println("Configuring Dorji for APRS transmit");
      nAddresses = 4;
      addresses[2].callsign = "WIDE1";
      addresses[2].ssid = 1;
      addresses[3].callsign = "WIDE2";
      addresses[3].ssid = 2;
      DorjiOn();
      delay(1000);
      DorjiSleep();
      delay(1000);
      DorjiWake();
      delay(5000);
      DorjiHighPower();
      delay(1000);
      UART_DORJI.print("AT+DMOCONNECT\r\n"); //handshake command
      delay(1000);
      UART_DORJI.print("AT+DMOSETGROUP=1,144.3900,144.3900,0000,4,0000\r\n"); //set tx and rx frequencies
      delay(1000);
      DorjiSleep();
      delay(3000);
      DorjiWake();
      delay(5000);
      nextState = TRANSMIT_APRS;
      break;

    case TRANSMIT_APRS:
      amBusy();
      for (int i = 0; i < 5; i++) {
        aprs_send(addresses, nAddresses
                , 27, 5, 50
                , goodLatitude, goodLongitude // degrees
                , 0 // meters
                , 0
                , 0
                , SYMBOL_TABLE
                , SYMBOL_CHAR
                , "VB EE Radio v8.1");
        delay(1000);
      }
      DorjiOff();
      FiveVOff();
      SupercapChargerOff();
      sendAPRSTimer = millis() + configAPRSInterval;
      nextState = DO_THE_THINGS;
      break;
    }
    previousState = currentState;
    currentState = nextState;
  }
}
