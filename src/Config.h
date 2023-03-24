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
#define MAX_BUFFER_SIZE 250

// Defines Constant for PWM Channels
// Used to identify which finger an object is to operate on
#define FINGER_THUMB_CHANNEL 0
#define FINGER_INDEX_CHANNEL 1
#define FINGER_MIDDLE_CHANNEL 2
#define FINGER_RING_CHANNEL 3
#define FINGER_PINKY_CHANNEL 4

// Safety Threshold for Servo Current Consumption
#define MAX_SERVO_CURRENT 300  // 300 mA maximum current draw per servo motor

// Defines constants for hand position selections
#define HAND_POSITION_PLATFORM_PUSH 0
#define HAND_POSITION_LARGE_DIAMETER 1
#define HAND_POSITION_INDEX_FINGER_POINTING 2

// Buffer structure for storing EMG data
struct buffer {
  int8_t myo1;
  int8_t myo2;
  int8_t myo3;
  int8_t myo4;
  int8_t myo5;
  int8_t myo6;
  int8_t myo7;
  int8_t myo8;
};