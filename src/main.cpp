/*
 *  Prosthetic Arm Control (P.A.c) ESP32S2 RTOS
 *
 *  Created on: 24 Nov, 2022
 *      Author: Zachary Roberts(zroberts1@ycp.edu)
 *
 *      This program is used to control the Prosthetic Arm created in
 *      collaboration with York College of Pennsylvania Capstone course.
 */

#include <../lib/ESP32AnalogRead/ESP32AnalogRead.h>
#include <Arduino.h>
#include <PACCurrentSense/PACCurrentSense.h>  // Current sense library for prosethic arm
#include <PACFSRFeedback/PACFSRFeedback.h>  // FSR Feedback library for prosethic arm
#include <PACServoDriver/PACServoDriver.h>  // Motor control libary for prosehtic arm

#include <deque>  // C++ libraries are not native to arduino, included within ESPIDF

#include "config.h"

// Creates Servo Driver and Current Sense Object
static PACServoDriver servoController;
static PACCurrentSense currentSense;
static PACFSRFeedback fsrFeedback;

// Creates ADC objects
static ESP32AnalogRead myoware1;  // Myoware Sensor Inside Fore arm
static ESP32AnalogRead myoware2;  // Myoware Sensor Outside Fore arm

// Buffer defintion for EMG Sensor
static std::deque<buffer> myoBuffer;  // Buffer for both myoware sensors

// Hysteresis toggle
static bool hysteresis = false;

/**
 * @brief Updates the position of the servo motors based on the result of the
 *        signal processing. Currently only a simple threshold comparision
 *        is implemented.
 *
 * @param pvParameter Void Pointer
 */
void updateServoMotors(void *pvParameter) {
  while (1) {
    // checks most recent sensor readings to determine the desired hand position
    if (myoBuffer.back().myo1 > 1000) {
      // sets hand to largeDiameter position
      servoController.largeDiameter();
    } else if (myoBuffer.back().myo2 > 1000) {
      // sets hand to indexFingerPointing position
      servoController.indexFingerPointing();
    } else {
      servoController.openHand();
    }

    // Updates servo position every 50 ms (20 Hz)
    vTaskDelay(50 * portTICK_PERIOD_MS);
  }
}

/**
 * @brief Reads the output of the electromyographic sensors and
 *        updates the buffers.
 *
 * @param pvParameter Void Pointer
 */
void readMyoSensor(void *pvParameter) {
  buffer tempBuffer;
  while (1) {
    // Checks if buffers are full and removes oldest data if necessary
    if (myoBuffer.size() >= MAX_BUFFER_SIZE) {
      myoBuffer.pop_front();
    }

    // Updates temp buffer value for each EMG sensor
    tempBuffer.myo1 = myoware1.readMilliVolts();
    tempBuffer.myo2 = myoware2.readMilliVolts();

    // Print myo input data for debuging purposes
    Serial.print("Myo1: ");
    Serial.println(tempBuffer.myo1);
    Serial.print("Myo2: ");
    Serial.println(tempBuffer.myo2);

    // updates buffer
    myoBuffer.push_back(tempBuffer);

    //  Delays the task for 10 ms (100 Hz)
    vTaskDelay(10 * portTICK_PERIOD_MS);
  }
}

/**
 * @brief chkHandCollision: Checks if any of the fingers have collided with and
 * object
 *
 * @param pvParameter Void Pointer
 */
void chkHandCollision(void *pvParameter) {
  while (1) {
    // Call functions to read in FSR values and convert to grams
    fsrFeedback.readAllFingerFeedback();
    fsrFeedback.calculateFeedbackGrams();

    // Parameters of FSR
    uint16_t margin = 50;
    uint16_t threshold = 200;

    // Hysteresis for read in FSR values
    if (hysteresis) {
      uint16_t marginThreshold = margin - threshold;
      if (fsrFeedback.indexFeedbackGrams <= (marginThreshold)) {
        hysteresis = false;
      } else if (fsrFeedback.middleFeedbackGrams <= (marginThreshold)) {
        hysteresis = false;
      } else if (fsrFeedback.thumbFeedbackGrams <= (marginThreshold)) {
        hysteresis = false;
      } else if (fsrFeedback.ringFeedbackGrams <= (marginThreshold)) {
        hysteresis = false;
      } else if (fsrFeedback.pinkyFeedbackGrams <= (marginThreshold)) {
        hysteresis = false;
      }
    } else {
      uint16_t marginThreshold = margin + threshold;
      if (fsrFeedback.indexFeedbackGrams >= (marginThreshold)) {
        hysteresis = true;
      } else if (fsrFeedback.middleFeedbackGrams >= (marginThreshold)) {
        hysteresis = true;
      } else if (fsrFeedback.thumbFeedbackGrams >= (marginThreshold)) {
        hysteresis = true;
      } else if (fsrFeedback.ringFeedbackGrams >= (marginThreshold)) {
        hysteresis = true;
      } else if (fsrFeedback.pinkyFeedbackGrams >= (marginThreshold)) {
        hysteresis = true;
      }
    }

    // Delays the task for 100 ms (10 Hz)
    vTaskDelay(100 * portTICK_PERIOD_MS);
  }
}

/**
 * @brief Sums the current of all servo motors, blocks all servo motors if it
 *        total current is greater than predefined MAX_SERVO_CURRENT
 *
 * @param pvParameter Void Pointer
 */
void chkMotorCurrent(void *pvParameter) {
  while (1) {
    // read current draw from all fingers
    currentSense.readAllFingerCurrents();

    // Compare total current to safety threshold
    if (currentSense.calculateTotalCurrent() > MAX_SERVO_CURRENT) {
      // Emergency Stop if current draw surpasses safety threshold
      servoController.stopAllMotion();
    }

    //  Delays the task for 10 ms (100 Hz)
    vTaskDelay(10 * portTICK_PERIOD_MS);
  }
}

/**
 * @brief Sets up configures and initializes all necessary objects and tasks
 *
 */
void setup() {
  // Starts serial communication at 115200 baud rate
  Serial.begin(115200);

  // Set CPU clock to 80MHz
  setCpuFrequencyMhz(80);  // Can be set to 80, 160, or 240 MHZ

  // Attach ADC object to GPIO pins
  myoware1.attach(8);  // Myoware 1 is attached to GPIO 1
  myoware2.attach(7);  // Myoware 2 is attached to GPIO 2

  // Create RTOS Tasks
  TaskHandle_t xHandle = NULL;

  xTaskCreatePinnedToCore(     // Use xTaskCreate() in vanilla FreeRTOS
      updateServoMotors,       // Function to be called
      "Update hand position",  // Name of task
      2048,                    // Stack size (bytes in ESP32, words in FreeRTOS)
      NULL,                    // Parameter to pass to function
      3,                       // Task priority (0 to configMAX_PRIORITIES - 1)
      &xHandle,                // Task handle
      0);                      // Run on core 1

  xTaskCreatePinnedToCore(     // Use xTaskCreate() in vanilla FreeRTOS
      readMyoSensor,           // Function to be called
      "Read Myoware Sensors",  // Name of task
      2048,                    // Stack size (bytes in ESP32, words in FreeRTOS)
      NULL,                    // Parameter to pass to function
      3,                       // Task priority (0 to configMAX_PRIORITIES - 1)
      &xHandle,                // Task handle
      0);                      // Run on core 1

  xTaskCreatePinnedToCore(    // Use xTaskCreate() in vanilla FreeRTOS
      chkHandCollision,       // Function to be called
      "Check Hand Collsion",  // Name of task
      2048,                   // Stack size (bytes in ESP32, words in FreeRTOS)
      NULL,                   // Parameter to pass to function
      2,                      // Task priority (0 to configMAX_PRIORITIES - 1)
      &xHandle,               // Task handle
      0);                     // Run on core 1

  xTaskCreatePinnedToCore(    // Use xTaskCreate() in vanilla FreeRTOS
      chkMotorCurrent,        // Function to be called
      "Check motor current",  // Name of task
      2048,                   // Stack size (bytes in ESP32, words in FreeRTOS)
      NULL,                   // Parameter to pass to function
      2,                      // Task priority (0 to configMAX_PRIORITIES - 1)
      &xHandle,               // Task handle
      0);                     // Run on core 1

  // TODO: Create task for data processing
}

void loop() {}