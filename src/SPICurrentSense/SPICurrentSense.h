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
#define V_REF 3
#define RESOLUTION 4096

class SPICurrentSense {
 private:
  // spi configuration
  static const int spiClk = 10000000;  // 10 MHz
  SPIClass* vspi = NULL;

  // ADC response
  uint8_t result1;
  uint8_t result2;
  uint16_t rawResult;

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
};