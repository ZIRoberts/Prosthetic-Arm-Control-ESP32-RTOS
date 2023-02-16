#pragma once

#include <Arduino.h>

// General PWM Configuration
#define PWM_FREQ 333      // Sets 333 Hz frequency for PWM
#define PWM_RESOLUTION 8  // Sets 8 bit resolution for PWM

// Defines PWM Pins
#define FINGER_THUMB_GPIO 42
#define FINGER_INDEX_GPIO 41
#define FINGER_MIDDLE_GPIO 40
#define FINGER_RING_GPIO 39
#define FINGER_PINKY_GPIO 38

// Defines Constant for PWM Channels
#define FINGER_THUMB_CHANNEL 0
#define FINGER_INDEX_CHANNEL 1
#define FINGER_MIDDLE_CHANNEL 2
#define FINGER_RING_CHANNEL 3
#define FINGER_PINKY_CHANNEL 4

// Defines Drive limits for servos
#define SERVO_HOME 50  // 20% duty cycle @8 bit resolution
#define SERVO_Max 205  // 80% duty cycle @8 bit resolution
#define SERVO_STOP 0   // Servos Stop moving and freeze in place

// Max Distance Drivable
// This limits the distance each finger can drive as each finger in unique
// and takes a different amount of rotation to actuate
// This is unique for each individual arm even if using the same model
#define FINGER_PINKY_DRIVE_LIMIT 190
#define FINGER_RING_DRIVE_LIMIT 195
#define FINGER_MIDDLE_DRIVE_LIMIT 180
#define FINGER_INDEX_DRIVE_LIMIT 205
#define FINGER_THUMB_DRIVE_LIMIT 205

class PACServoDriver {
 private:
  // Private variables
  uint8_t driveLimit;

  // Utility functions
  void setDutyCycle(uint8_t finger, uint8_t dutyCycle);
  uint8_t getDriveLimit(uint8_t finger);
  bool chkBlocked(uint8_t finger);

 public:
  // Initialization
  PACServoDriver();

  // Variables
  uint8_t driveSpeed;
  bool thumbBlocked;
  bool indexBlocked;
  bool middleBlocked;
  bool ringBlocked;
  bool pinkyBlocked;

  // Hand position functions
  void openHand();
  void largeDiameter();
  void indexFingerPointing();

  // TODO: Add remaining hand positions
  // void lateralPinch();
  // void largeDiameter();
  // void adductedThumb();
  // void thumbIndexFinger();
  // void tripod();
  // void prehensileSphere();

  // Safety functions
  void stopAllMotion();

  // utility functions
  void setDriveSpeed(uint8_t speed);
};