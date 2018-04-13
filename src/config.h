#ifndef CONFIG_H
#define CONFIG_H

//#define PRINT_ENCODED_DATA
//#define PRINT_TIMING
//#define PRINT_RSSI
//#define PRINT_DEBUG

#define MAX_MSG_LENGTH 15
#define NPAR 15
#define FRAME_SIZE ( MAX_MSG_LENGTH + NPAR )

#define GFSK_SDN 6
#define GFSK_IRQ 2
#define GFSK_GATE 5
#define GFSK_GPIO_0 7
#define GFSK_GPIO_1 8
#define GFSK_GPIO_2 20
#define GFSK_GPIO_3 21
#define GFSK_CS 22
#define GFSK_SCK 13
#define GFSK_MOSI 11
#define GFSK_MISO 12

#define RF_FREQ 433.5
#define RF_MODE RH_RF24::GFSK_Rb5Fd10

// FSK_Rb0_5Fd1   // FSK  500 bps
// FSK_Rb5Fd10     // FSK  5   kbps
// FSK_Rb50Fd100   // FSK  50  kbps
// FSK_Rb150Fd300  // FSK  150 kbps
// GFSK_Rb0_5Fd1   // GFSK 500 bps
// GFSK_Rb5Fd10    // GFSK 5   kbps
// GFSK_Rb50Fd100  // GFSK 50  kbps
// GFSK_Rb150Fd300 // GFSK 150 kbps
// OOK_Rb5Bw30     // OOK  5   kbps
// OOK_Rb10Bw40   // OOK  10  kbps

#define WATCHDOG_PIN 15
#define VS1 A4
#define VS2 A5
#define SKYBASS_SERIAL Serial1
#define STRATO_SERIAL Serial2

#endif