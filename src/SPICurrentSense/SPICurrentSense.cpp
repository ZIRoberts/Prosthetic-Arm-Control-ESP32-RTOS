/*
 *  Prosthetic Arm Control (P.A.C) SPI Current Sense
 *
 *  Created on: 26 Feb, 2023
 *      Author: Zachary Roberts
 *
 *      This library is used to manage the communication with the MAX11629EEE+
 *      ADC, conversions, and calculations with the data from the Low End
 *      Current sense circuit for each respective finger on the prosthetic arm
 *      created in collaboration with York College of Pennsylvania
 *      Capstone Course
 */

#include "SPICurrentSense.h"

/**
 * @brief Construct a new SPICurrentSense::SPICurrentSense object
 *
 */
SPICurrentSense::SPICurrentSense() {
  // Initializes instance of SPIClass attached to VSPI
  delay(1);  // 1 ms second delay for ADC to boot
  vspi = new SPIClass(VSPI);
  vspi->begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS);
  pinMode(EOC, INPUT);

  // set up slave select pins as outputs as the Arduino API
  // doesn't handle automatically pulling SS low
  pinMode(vspi->pinSS(), OUTPUT);  // VSPI SS

  // Configured ADC behavior
  SPITransmit(RESET_REG);  // Clears all ADC registers
  // Use internal clock and external reference for conversions
  SPITransmit(INTERNAL_CLK | EXTERNAL_VREF);
  SPITransmit(AVG_DISABLE);  // Disable averaging, single conversion mode
}

/**
 * @brief Transmits instructions to external ADC over SPI
 *
 * @param data unsigned 8 bit integer containing corresponding instruction
 * @return uint8_t response from external ADC
 */
uint8_t SPICurrentSense::SPITransmit(byte data) {
  uint8_t response;
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));

  // pull SS slow to prep other end for transfer
  digitalWrite(vspi->pinSS(), LOW);

  // Transmits SPI data
  response = vspi->transfer(data);

  // pull SS High to signify end of data transfer
  digitalWrite(vspi->pinSS(), HIGH);
  vspi->endTransaction();

  return response;
}

/**
 * @brief Reads current status of all current sensors. Converts raw data to
 *        current in milliAmperes. Each reading is saved in it's respective
 *        public variable
 */
void SPICurrentSense::readAllFingerCurrents() {
  // Requests signal data from all current sensors
  SPITransmit(AIN4 | SCAN_0N);  // Scans channels 0 through 4

  // Wait for all readings to be returned
  while (1) {
    // Checks for conversions to be finished
    if (digitalRead(EOC) == 0) {
      // Serial.println("low"); // debugging check for EOC pin
      // Reads
      for (uint8_t i = 0; i < 5; i++) {
        // Transmit no instructions, read ADC response
        result1 = SPITransmit(READ_ONLY);
        result2 = SPITransmit(READ_ONLY);

        // Converts the two 8 bit responses to a single 16 bit value
        rawResult = (result1 << 8) | result2;

        // Converts raw data to voltage representation in mV
        rawResult = rawResult * 3000 / RESOLUTION;

        // Calculates and saves current to corresponding finger
        switch (i) {
          case 0:  // Thumb
            thumbCurrent = (rawResult / 2.1) - thumbOffset;
            break;
          case 1:  // Index finger
            indexCurrent = (rawResult / 2.1) - thumbOffset;
            break;
          case 2:  // Middle finger
            middleCurrent = (rawResult / 2.1) - thumbOffset;
            break;
          case 3:  // Ring finger
            ringCurrent = (rawResult / 2.1) - thumbOffset;
            break;
          case 4:  // Pinky finger
            pinkyCurrent = (rawResult / 2.1) - thumbOffset;
            break;
        }
      }
      // Breaks out of loop when ADC response is received
      break;
    }
  }
}

/**
 * @brief Adds all current values to calculate total servo current draw. Must
 *        call readAllFIngerCurrents() first to ensure most up to date readings
 *
 * @return uint16_t Total combined current value of all current sense circuits
 */
uint16_t SPICurrentSense::calculateTotalCurrent() {
  totalCurrent =
      thumbCurrent + indexCurrent + middleCurrent + ringCurrent + pinkyCurrent;

  return totalCurrent;
}

/**
 * @brief
 *
 * @param tmpThumbOffset
 * @param tmpIndexOffset
 * @param tmpMiddleOffset
 * @param tmpRingOffset
 * @param tmpPinkyOffset
 */
void SPICurrentSense::setCalibrationOffsets(uint16_t tmpThumbOffset,
                                            uint16_t tmpIndexOffset,
                                            uint16_t tmpMiddleOffset,
                                            uint16_t tmpRingOffset,
                                            uint16_t tmpPinkyOffset) {
  thumbOffset = tmpThumbOffset;
  indexOffset = tmpIndexOffset;
  middleOffset = tmpMiddleOffset;
  ringOffset = tmpRingOffset;
  pinkyOffset = tmpPinkyOffset;
}