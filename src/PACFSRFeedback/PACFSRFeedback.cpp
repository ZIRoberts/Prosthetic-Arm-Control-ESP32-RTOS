/*
 *  Prosthetic Arm Control (P.A.C) Current Sense
 *
 *  Created on: 02 Feb, 2023
 *      Author: Zachary Roberts
 *
 *      This library is used to manage the ADC readings, conversions, and
 *      calculations with the data from the Force Sensetive Resistor (FSR)
 *      feedback circuit for each respective fingers on the prosthetic arm
 *      created in collaboration with York College of Pennsylvania Capstone
 *      Course
 */

#include "PACFSRFeedback.h"

/**
 * @brief Construct a new PACCurrentSense::PACCurrentSense object
 *        Attaches ADC objects to the corresponding current sense circuit pins
 *        Initializes current readings to 0 mv
 */
PACFSRFeedback::PACFSRFeedback() {
  // Attach Feedback sensors to GPIO pins
  thumbFeedback.attach(14);
  indexFeedback.attach(13);
  middleFeedback.attach(10);
  ringFeedback.attach(5);
  pinkyFeedback.attach(3);

  // initializes feedback readings to 0
  thumbFeedbackGrams = 0;
  indexFeedbackGrams = 0;
  middleFeedbackGrams = 0;
  ringFeedbackGrams = 0;
  pinkyFeedbackGrams = 0;
}

/**
 * @brief Calculates the force applied on each FSR in grams using the raw input
 *        voltage measured from the FSR feedback circuit.
 */
void PACFSRFeedback::calculateFeedbackGrams() {
  // Voltage can be converted to force in grams using the fallowing equation
  // grams = pow((271/(47000*((3.3/(miliVolts/1000)) -1 ))),(1/0.69))

  thumbFeedbackGrams = pow(
      (271 / (47000 * ((3.3 / (thumbFeedbackRaw / 1000)) - 1))), (1 / 0.69));
  indexFeedbackGrams = pow(
      (271 / (47000 * ((3.3 / (indexFeedbackRaw / 1000)) - 1))), (1 / 0.69));
  middleFeedbackGrams = pow(
      (271 / (47000 * ((3.3 / (middleFeedbackRaw / 1000)) - 1))), (1 / 0.69));
  ringFeedbackGrams =
      pow((271 / (47000 * ((3.3 / (ringFeedbackRaw / 1000)) - 1))), (1 / 0.69));
  pinkyFeedbackGrams = pow(
      (271 / (47000 * ((3.3 / (pinkyFeedbackRaw / 1000)) - 1))), (1 / 0.69));
}

/**
 * @brief Reads the feedback voltage from the force sensitive resistors
 *        Sets the corresponding global variables
 */
void PACFSRFeedback::readAllFingerFeedback() {
  thumbFeedbackRaw = senseFeedback(FINGER_THUMB_CHANNEL);
  indexFeedbackRaw = senseFeedback(FINGER_INDEX_CHANNEL);
  middleFeedbackRaw = senseFeedback(FINGER_MIDDLE_CHANNEL);
  ringFeedbackRaw = senseFeedback(FINGER_RING_CHANNEL);
  pinkyFeedbackRaw = senseFeedback(FINGER_PINKY_CHANNEL);
}

/**
 * @brief Senses the current in millivolts from the FSR feedback circuit
 *
 * @param finger number corresponding to the finger to be measured
 * @return uint16_t current measured in millivolts
 */
uint16_t PACFSRFeedback::senseFeedback(uint8_t finger) {
  switch (finger) {
    case 0:  // Thumb
      return indexFeedback.readMilliVolts();
      break;
    case 1:  // Index finger
      return middleFeedback.readMilliVolts();
      break;
    case 2:  // Middle finger
      return thumbFeedback.readMilliVolts();
      break;
    case 3:  // Ring finger
      return ringFeedback.readMilliVolts();
      break;
    case 4:  // Pinky finger
      return pinkyFeedback.readMilliVolts();
      break;
  }

  // returns max 16 bit value if the requested finger is undefined will block
  // all operations from occurring on the arm until resolved.
  return 65535;
}
