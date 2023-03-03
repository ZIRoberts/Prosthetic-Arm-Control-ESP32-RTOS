/*
 *  Prosthetic Arm Control (P.A.C) MAX11529 SPI Commands
 *
 *  Created on: 26 Feb, 2023
 *      Author: Zachary Roberts
 *
 *      This library is used to store and define all of the SPI commands
 *      required to communicate with the MAX11629EEE+ external SPI ADC. This
 *      library was created for use with the prosthetic arm created in
 *      collaboration with York College of Pennsylvania Capstone Course
 */

/*******************************************************************************
Conversion Register
*******************************************************************************/
// Select Input Channel (N)
#define AIN0 0b10000000  // Read input from channel 0
#define AIN1 0b10001000  // Read input from channel 1
#define AIN2 0b10010000  // Read input from channel 2
#define AIN3 0b10011000  // Read input from channel 3
#define AIN4 0b10100000  // Read input from channel 4
#define AIN5 0b10101000  // Read input from channel 5
#define AIN6 0b10110000  // Read input from channel 6
#define AIN7 0b10111000  // Read input from channel 7

// Select Scan Mode
#define SCAN_0N 0b10000000   // Scans input from channel 0 through N
#define SCAN_N7 0b10000010   // Scans input from channel N through 7
#define SCAN_AVG 0b10000100  // Scans channel N repeatedly
#define SCAN_N 0b10000110    // Scan input from channel N once

/*******************************************************************************
Setup Register
*******************************************************************************/
// Sampling Clock Select pin 8 = CVNST
#define INTERNAL_CVST 0b01000000  // Conversion & acquisition internally clock
// Conversion internally clock, Acquisition externally clock
#define INTERNAL_EXTERNAL_CVST 0b010001000

// Sampling Clock Select pin 8 = AIN7
#define INTERNAL_CLK 0b01001000  // Conversion & acquisition internally clock
#define EXTERNAL_CLK 0b01001100  // Conversion & acquisition externally clock

// Voltage Reference Select
#define INTERNAL_WAKEUP 0b01000000  // Internal reference, wake-up delay
#define EXTERNAL_VREF 0b01000100    // External reference, no wake up delay
#define INTERNAL_VREF 0b01001100    // External reference, no wake up delay

/*******************************************************************************
Averaging Register
*******************************************************************************/
// Disable Averaging
#define AVG_DISABLE 0b00100000

// Enable Averaging
#define AVG_4_SAMPLES 0b00110000   // Perform 4 conversions and averages result
#define AVG_8_SAMPLES 0b00110100   // Perform 8 conversions and averages result
#define AVG_16_SAMPLES 0b00111000  // Perform 16 conversions and averages result
#define AVG_32_SAMPLES 0b00111100  // Perform 32 conversions and averages result

// Number of Scans (ONLY applicable of SCAN MODE 10 is set)
#define NSCAN_4 0b00000000   // Scans channel N and returns 4 results
#define NSCAN_8 0b00000001   // Scans channel N and returns 8 results
#define NSCAN_16 0b00000010  // Scans channel N and returns 16 results
#define NSCAN_32 0b00000011  // Scans channel N and returns 32 results

/*******************************************************************************
Reset Register
*******************************************************************************/
// Resets all registers
#define RESET_REG 0b00010000
// Resets FIFO registers only
#define RESET_FIFO 0b00011000

/*******************************************************************************
additional commands
*******************************************************************************/
#define READ_ONLY 0b00000000