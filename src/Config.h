/*
 *  Prosthetic Arm Control (P.A.C) Current Sense
 *
 *  Created on: 06 Dec, 2022
 *      Author: Zachary Roberts (zroberts1@ycp.edu)
 *
 *      This file is used to store defines and set configurations for the main
 *      proccess of the application.
 */

// Sets max buffer size to 5 seconds, 1 sec = 100 samples
#define MAX_BUFFER_SIZE 499

// Defines Constant for PWM Channels
// Used to identify which finger an object is to operate on
#define FINGER_THUMB_CHANNEL 0
#define FINGER_INDEX_CHANNEL 1
#define FINGER_MIDDLE_CHANNEL 2
#define FINGER_RING_CHANNEL 3
#define FINGER_PINKY_CHANNEL 4

// Saftey Threshold for Servo Current Consumption
#define MAX_SERVO_CURRENT 930  // 300 mA maximum current draw per servo motor

// Buffer structure for storing EMG data
struct buffer {
  uint16_t myo1;
  uint16_t myo2;
  //   uint16_t myo3;
  //   uint16_t myo4;
  //   uint16_t myo5;
  //   uint16_t myo6;
  //   uint16_t myo7;
  //   uint16_t myo8;
};