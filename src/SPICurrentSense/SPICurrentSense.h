#pragma once

#include <Arduino.h>
#include <SPI.h>
// Include bit field SPI commands
#include "MAX11629_SPICommands.h"

// Used to identify which finger an object is to operate on
#define FINGER_THUMB_CHANNEL 0
#define FINGER_INDEX_CHANNEL 1
#define FINGER_MIDDLE_CHANNEL 2
#define FINGER_RING_CHANNEL 3
#define FINGER_PINKY_CHANNEL 4

// SPI Configuration
#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#define VSPI
#endif

// SPI Pins
#define VSPI_MOSI 35
#define VSPI_MISO 37
#define VSPI_SCLK 36
#define VSPI_SS 34

// End of Conversion pin
#define EOC 33

// ADC configruation
#define V_REF_3V 3000  // 3 V regerence = 3000 mV
#define RESOLUTION 4096

class SPICurrentSense {
 private:
  // spi configuration
  static const int spiClk = 10000000;  // 10 MHz
  SPIClass* vspi = NULL;

  // ADC response
  uint8_t result1;
  uint8_t result2;
  int rawResult;

  // Current Sense Calibration offset values
  uint16_t thumbOffset;
  uint16_t indexOffset;
  uint16_t middleOffset;
  uint16_t ringOffset;
  uint16_t pinkyOffset;

 public:
  // Constructor
  SPICurrentSense();

  // stores ADC reading current sensor
  uint16_t thumbCurrent;
  uint16_t indexCurrent;
  uint16_t middleCurrent;
  uint16_t ringCurrent;
  uint16_t pinkyCurrent;
  uint16_t totalCurrent;

  // function definitions
  uint8_t SPITransmit(byte data);
  void readAllFingerCurrents();
  uint16_t calculateTotalCurrent();
  void setCalibrationOffsets(uint16_t tmpThumbOffset, uint16_t tmpIndexOffset,
                             uint16_t tmpMiddleOffset, uint16_t tmpRingOffset,
                             uint16_t tmpPinkyOffset);
};