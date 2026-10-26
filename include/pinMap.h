/**
 * @file pinMap.h
 * @author ChiHo Ngan, Red Bird Racing
 * @brief Pin assignments for the front dash PCBs.
 * @details Define OLD_PCB (env "old_pcb") for the pre-SD test board: no SD, no
 * power-fail input, no CAN2, MCP2515 INT pins not wired.
 * @version 1.0
 * @date 2026-10-27
 */
#ifndef PIN_MAP_H
#define PIN_MAP_H

#ifdef OLD_PCB
#define CAN0_CS        PIN_PB2
#define CAN1_CS        PIN_PB1
#define PIN_ENC_A      PIN_PD2 // INT0
#define PIN_ENC_B      PIN_PC1 // PCINT9
#define ENC_B_PINREG   PINC
#define ENC_B_BIT      PC1
#define HC12_SET       PIN_PD3
#else
// GEN6 front dash PCB, U1 = ATmega328 TQFP-32
#define CAN0_CS        PIN_PD5 // pad 9
#define CAN1_CS        PIN_PD6 // pad 10
#define CAN2_CS        PIN_PD7 // pad 11
#define CAN_INT_BITS   {PC1, PC2, PC3} // MCP2515 INT0..2 on PINC, active low
#define SD_CS          PIN_PD4 // pad 2
#define GPIO_1         PIN_PB1
#define GPIO_2         PIN_PB2
#define PIN_ENC_A      PIN_PD2 // pad 32, INT0
#define PIN_ENC_B      PIN_PD3 // pad 1, INT1
#define ENC_B_PINREG   PIND
#define ENC_B_BIT      PD3
#define PIN_ENC_BTN    PIN_PB0 // pad 12
#define HC12_SET       PIN_PC0 // pad 23
#endif

#endif // PIN_MAP_H
