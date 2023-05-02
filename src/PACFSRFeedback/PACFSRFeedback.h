#pragma once

#include <../lib/ESP32AnalogRead/ESP32AnalogRead.h>
#include <Arduino.h>

// Defines Constant for PWM Channels
// Used to identify which finger an object is to operate on
#define FINGER_THUMB_CHANNEL 0
#define FINGER_INDEX_CHANNEL 1
#define FINGER_MIDDLE_CHANNEL 2
#define FINGER_RING_CHANNEL 3
#define FINGER_PINKY_CHANNEL 4
#define PALM_CHANNEL 5

class PACFSRFeedback {
 private:
  // ADC Objects
  ESP32AnalogRead indexFeedback;
  ESP32AnalogRead middleFeedback;
  ESP32AnalogRead thumbFeedback;
  ESP32AnalogRead ringFeedback;
  ESP32AnalogRead pinkyFeedback;
  ESP32AnalogRead palmFeedback;

  // FSR Input in Ohms
  // double thumbFeedbackRaw;
  // double indexFeedbackRaw;
  // double middleFeedbackRaw;
  // double ringFeedbackRaw;
  // double pinkyFeedbackRaw;
  // double palmFeedbackRaw;

 public:
  // Constructor
  PACFSRFeedback();

  // FSR Input converted from Ohms to Grams
  double thumbFeedbackGrams;
  double indexFeedbackGrams;
  double middleFeedbackGrams;
  double ringFeedbackGrams;
  double pinkyFeedbackGrams;
  double palmFeedbackGrams;

  double thumbFeedbackRaw;
  double indexFeedbackRaw;
  double middleFeedbackRaw;
  double ringFeedbackRaw;
  double pinkyFeedbackRaw;
  double palmFeedbackRaw;

  void readAllFingerFeedback();
  void calculateFeedbackGrams();
  uint16_t senseFeedback(uint8_t finger);
};
