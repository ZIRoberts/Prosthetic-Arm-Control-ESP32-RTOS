/*
 *  Prosthetic Arm Control (P.A.C) Motor Driver
 *
 *  Created on: 24 Nov, 2022
 *      Author: Zachary Roberts (zroberts1@ycp.edu)
 *
 *      This library is used to manage the location, hand postion, and drive
 *      characteristics of the servo motors controlling each finger of the
 *      prosthetic arm created in collaboration with York College of
 * Pennsylvania Capstone Course
 */
#include "PACServoDriver.h"

/**
 * @brief Construct a new PACServoDriver::PACServoDriver object
 *        Initializes PWM channels and attaches to respective pin
 *        Drives hand to open state on start up
 */
PACServoDriver::PACServoDriver() {
  // Configures PWM Frequency and Resolution for all channels
  for (int channel = 0; channel < 5; channel++) {
    ledcSetup(channel, PWM_FREQ, PWM_RESOLUTION);
  }

  // Attaches PWM pins to Motor Control pins
  ledcAttachPin(FINGER_THUMB_GPIO, FINGER_THUMB_CHANNEL);
  ledcAttachPin(FINGER_INDEX_GPIO, FINGER_INDEX_CHANNEL);
  ledcAttachPin(FINGER_MIDDLE_GPIO, FINGER_MIDDLE_CHANNEL);
  ledcAttachPin(FINGER_RING_GPIO, FINGER_RING_CHANNEL);
  ledcAttachPin(FINGER_PINKY_GPIO, FINGER_PINKY_CHANNEL);

  // Used to determine if an object is within the hand
  thumbBlocked = false;
  indexBlocked = false;
  middleBlocked = false;
  ringBlocked = false;
  pinkyBlocked = false;

  // Arm initialize to an open hand position
  openHand();
}

/*******************************************************************************
 * Hand position functions
 *******************************************************************************/

/**
 * @brief All fingers are set to open, all fingers are extended
 *
 */
void PACServoDriver::openHand() {
  for (int i = 0; i < 5; i++) {
    setDutyCycle(i, SERVO_HOME);
  }
}

/**
 * @brief Hand closes until all fingers are closed or an object is within the
 * hand
 *
 */
void PACServoDriver::largeDiameter() {
  for (int i = 0; i < 5; i++) {
    setDutyCycle(i, SERVO_Max);
  }
}

/**
 * @brief All fingers close expect the index finger
 *
 */
void PACServoDriver::indexFingerPointing() {
  for (int i = 0; i < 5; i++) {
    if (i != FINGER_INDEX_CHANNEL) {
      setDutyCycle(i, SERVO_Max);
    }
  }
}

/*******************************************************************************
 * Safety Functions
 *******************************************************************************/

/**
 * @brief  Stops all motors by setting the duty cycle of each motor to zero.
 *         motors will remain in current position until instructed otherwise
 */
void PACServoDriver::stopAllMotion() {
  for (int i = 0; i < 5; i++) {
    setDutyCycle(i, SERVO_STOP);
  }
}

/*******************************************************************************
 * Utility Functions
 *******************************************************************************/

/**
 * @brief Sets the duty cycle of the specified finger. Checks if the finger is
 *        blocked and ensures the finger is not driven past it's specified
 *        maximum distance.
 *
 * @param finger PWM channel of the specified finger
 * @param dutyCycle Positive duty cycle corresponding the desired distance
 */
void PACServoDriver::setDutyCycle(uint8_t finger, uint8_t dutyCycle) {
  // checks to see if the movement of the finger is blocked
  if (!chkBlocked(finger)) {
    // retrieves the drive limit of the specified finger
    driveLimit = getDriveLimit(finger);

    // Drives finger to desired distance or stops at specified maximum
    if (dutyCycle < driveLimit) {
      ledcWrite(finger, dutyCycle);
    } else {
      ledcWrite(finger, driveLimit);
    }
  }
}

/**
 * @brief Returns the maximum duty cycle for the specified finger
 *
 * @param finger PWM channel number corresponding to the specified finger
 * @return uint8_t Maximum duty cycle for specified finger
 */
uint8_t PACServoDriver::getDriveLimit(uint8_t finger) {
  switch (finger) {
    case 0:
      return FINGER_THUMB_DRIVE_LIMIT;
      break;
    case 1:
      return FINGER_INDEX_DRIVE_LIMIT;
      break;
    case 2:
      return FINGER_MIDDLE_DRIVE_LIMIT;
      break;
    case 3:
      return FINGER_RING_DRIVE_LIMIT;
      break;
    case 4:
      return FINGER_PINKY_DRIVE_LIMIT;
      break;
  }

  // Returns home position for any unknown finger
  return SERVO_HOME;
}

/**
 * @brief Returns if the movement of the specified finger is blocked or
 * unblocked. Returns true (blocked) if the finger is unknown
 *
 * @param finger PWM channel number corresponding to the specified finger
 * @return true Returned when the motion of the finger is blocked by an object
 * @return false Returned when then finger can move freely
 */
bool PACServoDriver::chkBlocked(uint8_t finger) {
  switch (finger) {
    case 0:
      return thumbBlocked;
      break;
    case 1:
      return indexBlocked;
      break;
    case 2:
      return middleBlocked;
      break;
    case 3:
      return ringBlocked;
      break;
    case 4:
      return pinkyBlocked;
      break;
  }

  // If finger is not know, return true to prevent finger from being driven
  return true;
}