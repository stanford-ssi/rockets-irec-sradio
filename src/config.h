#ifndef CONFIG_H
#define CONFIG_H

#define PRINT_ENCODED_DATA
#define PRINT_TIMING

#define MAX_MSG_LENGTH 50
#define NPAR 40
#define FRAME_SIZE ( MAX_MSG_LENGTH + NPAR )

#define GFSK_SDN 6
#define GFSK_IRQ 2
#define GFSK_GATE 5
#define GFSK_GPIO_0 21
#define GFSK_GPIO_1 20
#define GFSK_GPIO_2 7
#define GFSK_GPIO_3 8
#define GFSK_CS 25
#define GFSK_SCK 13
#define GFSK_MOSI 11
#define GFSK_MISO 12

#define RF_FREQ 433.0
#define RF_MODE RH_RF24::GFSK_Rb0_5Fd1

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

#endif