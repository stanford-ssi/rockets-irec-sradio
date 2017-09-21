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
 More testing.
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


const int MESSAGE_LENGTH = 35;
#define NPAR 25

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

// Constants and Tuneable Variables
const int     DORJI_BAUD =             9600;
const int     DRA818V_PTT_DELAY =      150;             // Milliseconds to wait after PTT to transmit
const int     DRA818V_COMMAND_DELAY =  250;             // Milliseconds to wait after sending a UART command
const int     DORJI_WAKEUP_INT =       5000;            // Milliseconds to wake dorji up
#define       S_CALLSIGN               "KK6MIR"         // FCC Callsign
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
        WAIT_FOR_COMMAND,
        GFSK_CONFIG,
        TRANSMIT_GFSK
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



void highBoostPower(){
    Wire.beginTransmission(0x2E);
    Wire.write(byte(0x10));
    Wire.endTransmission();
}

/*************************************************************************************************************************************************************************************************/

void lowBoostPower(){
    Wire.beginTransmission(0x2E);
    Wire.write(byte(0x7F));
    Wire.endTransmission();
}

/*************************************************************************************************************************************************************************************************/

void SupercapChargerOff(){
  digitalWrite(BOOST_EN, LOW);
}

/*************************************************************************************************************************************************************************************************/

void SupercapChargerOn(){
  digitalWrite(BOOST_EN, HIGH);
}

/*************************************************************************************************************************************************************************************************/

void FiveVOff(){
  digitalWrite(EN_5V, LOW);
}

/*************************************************************************************************************************************************************************************************/

void FiveVOn(){
  digitalWrite(EN_5V, HIGH);
}

/*************************************************************************************************************************************************************************************************/

void DorjiOn(){
  digitalWrite(DORJI_GATE, LOW);
}

/*************************************************************************************************************************************************************************************************/

void DorjiOff(){
  digitalWrite(DORJI_GATE, HIGH);
}

/*************************************************************************************************************************************************************************************************/

void DorjiSleep(){
  digitalWrite(DORJI_SLP, LOW);
}

/*************************************************************************************************************************************************************************************************/

void DorjiWake(){
  digitalWrite(DORJI_SLP, HIGH);
}

/*************************************************************************************************************************************************************************************************/

void DorjiLowPower(){
  digitalWrite(DORJI_PSEL, LOW);
}

/*************************************************************************************************************************************************************************************************/

void DorjiHighPower(){
  digitalWrite(DORJI_PSEL, HIGH);
}

/*************************************************************************************************************************************************************************************************/

void DorjiPTTOn(){
  digitalWrite(DORJI_PTT, LOW);
}

/*************************************************************************************************************************************************************************************************/

void DorjiPTTOff(){
  digitalWrite(DORJI_PTT, HIGH);
}

/*************************************************************************************************************************************************************************************************/

void GFSKOff(){
  digitalWrite(GFSK_GATE, HIGH);
}

/*************************************************************************************************************************************************************************************************/

void GFSKOn(){
  digitalWrite(GFSK_GATE, LOW);
}




void parse_radio_config(vb_rf_config* config) {
  switch (config->config) {
  case VB_FREQUENCY:
    Serial.print("   - Setting ValBal frequency to ");
    Serial.println(config->data[0]);
    break;
  case BEC_DIVIDER:
    Serial.print("   - Setting BEC divider to ");
    Serial.println(config->data[0]);
    break;
  case APRS_DIVIDER:
    Serial.print("   - Setting APRS divider to ");
    Serial.println(config->data[0]);
    break;
  }
}

uint8_t frame_size;
uint8_t latest_frame[256];

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
    Serial.print(" > Getting data frame. ");
    int sz = message.data[0];
    Serial.println(sz);
    memcpy(latest_frame, message.data+1, sz);
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


int main() {
  pinMode(BOOST_EN, OUTPUT);
  pinMode(EN_5V, OUTPUT);
  pinMode(DORJI_GATE, OUTPUT);
  pinMode(GFSK_GATE, OUTPUT);
  pinMode(DORJI_SLP, OUTPUT);
  pinMode(DORJI_PTT, OUTPUT);
  pinMode(DORJI_PSEL, OUTPUT);
  pinMode(GFSK_SDN,OUTPUT);
  SupercapChargerOff();
  FiveVOff();
  DorjiOff();
  GFSKOff();
  DorjiSleep();
  DorjiPTTOff();
  DorjiLowPower();
  Wire.begin();//I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
  //lowBoostPower();
  highBoostPower();
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
  delay(3000);
  Serial2.begin(VBRF_BAUD_RATE);

  Serial.println("sending gpio2 output");
  pinMode(PAYLOAD_4, OUTPUT);

  int sendRadioTimer = millis() + 6000;

  initialize_ecc();

  while (true) {
    switch (currentState) {
    case WAIT_FOR_COMMAND:
      if (RADIO_MODE == AUTOMATIC) {
        receive_byte();
      }
      if (millis() > sendRadioTimer) {
        nextState = TRANSMIT_GFSK;
      } else {
        nextState = WAIT_FOR_COMMAND;
      }
      break;
    case GFSK_CONFIG:
      GFSKOn();
      rf24.init(MESSAGE_LENGTH+NPAR);
      uint8_t buf[8];
      if (!rf24.command(RH_RF24_CMD_PART_INFO, 0, 0, buf, sizeof(buf))) {
        Serial.println("SPI ERROR");
      } else {
        Serial.println("SPI OK");
      }
      if (!rf24.setFrequency(433.5)) {
        Serial.println("setFrequency failed");
      } else {
        Serial.println("Frequency set to 433.5 MHz");
      }
      rf24.setModemConfig(rf24.GFSK_Rb0_5Fd1);   // GFSK 500 bps
      rf24.setTxPower(0x4f);
      nextState = WAIT_FOR_COMMAND;
      break;
    case TRANSMIT_GFSK:
      uint8_t data[256] = {0};
      uint8_t leng = MESSAGE_LENGTH + NPAR;

      if (frame_size >= MESSAGE_LENGTH) {
        Serial.println("WELL CRAP");
      }

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

      rf24.send(data, MESSAGE_LENGTH + NPAR);
      rf24.waitPacketSent();


      sendRadioTimer = millis() + 1500;

      nextState = WAIT_FOR_COMMAND;
      break;
    }
    previousState = currentState;
    currentState = nextState;
  }
}
