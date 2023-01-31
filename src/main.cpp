/*
 *  Prosthetic Arm Control (P.A.c) ESP32S2 RTOS
 *
 *  Created on: 24 Nov, 2022
 *      Author: Zachary Roberts(zroberts1@ycp.edu)
 *     
 *      This program is used to control the Prosthetic Arm created in 
 *      collaboration with York College of Pennsylvania Capstone course.
 */

#include "config.h"
#include <Arduino.h>
#include <../lib/ESP32AnalogRead/ESP32AnalogRead.h>
#include <PACServoDriver/PACServoDriver.h> // Motor control libary for prosehtic arm
#include <PACCurrentSense/PACCurrentSense.h> // Current sense library for prosethic arm
#include <deque> // C++ libraries are not native to arduino, included within ESPIDF

// Creates Servo Driver and Current Sense Object
static PACServoDriver servoController;
static PACCurrentSense currentSense; 

// Creates ADC objects
static ESP32AnalogRead myoware1; // Myoware Sensor Inside Fore arm
static ESP32AnalogRead myoware2; // Myoware Sensor Outside Fore arm
static ESP32AnalogRead thumbFeedback;
static ESP32AnalogRead indexFeedback;
static ESP32AnalogRead middleFeedback;
static ESP32AnalogRead ringFeedback;
static ESP32AnalogRead pinkyFeedback;
uint16_t totalCurrent;
// Buffer definitions
static std::deque<double> myo1Buffer; // Buffer for inside forearm Myoware Sensor
static std::deque<double> myo2Buffer; // Buffer for outside forearm Myoware Sensor

/**
 * @brief updateServoMotors: Updates the position of the servo motors based on the
 *        result of signal processing. Currently only a simple threshold comparision
 *        is implemented.
 * 
 * @param pvParameter Void Pointer 
 */
void updateServoMotors(void *pvParameter){
  while(1){
    //checks most recent sensor readings to determine the desired hand position
    if (myo1Buffer.back() > 1000){ 
      //sets hand to largeDiameter position
      servoController.largeDiameter();
    } else if (myo2Buffer.back() > 1000){
      // sets hand to indexFingerPointing position
      servoController.indexFingerPointing();
    } else{
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
void readMyoSensor(void *pvParameter){
  while(1){
    // Checks if buffers are full and removes oldest data if necessary
    if (myo1Buffer.size() >= MAX_DUFFER_SIZE){
      myo1Buffer.pop_front();
    }

    if (myo2Buffer.size() >= MAX_DUFFER_SIZE){
      myo2Buffer.pop_front();
    }

    // Updates buffers for each myoelectrographic sensor
    myo1Buffer.push_back(myoware1.readMilliVolts());
    myo2Buffer.push_back(myoware2.readMilliVolts());

    //Serial.println(myo1Buffer.back());

    // Delays the task for 10 ms (100 Hz)
    vTaskDelay(10 * portTICK_PERIOD_MS );
  }
}

/**
 * @brief chkHandCollision: Checks if any of the fingers have collided with and object
 * 
 * @param pvParameter Void Pointer 
 */
void chkHandCollision(void *pvParameter){
   while(1){ 
    //TODO: Add Hysteresis to the feedback sensor
    // Voltage can be converted to force in grams using the fallowing equation
    // grams = pow((271/(47000*((3.3/(miliVolts/1000)) -1 ))),(1/0.69))


    // Checks if any of the fingers have collided with an object
    if (thumbFeedback.readMilliVolts() > FEEDBACK_THRESHOLD){
      servoController.thumbBlocked = true;
    } else{
      servoController.thumbBlocked = false;
    }

    if (indexFeedback.readMilliVolts() > FEEDBACK_THRESHOLD){
      servoController.indexBlocked = true;
    } else{
      servoController.indexBlocked = false;
    }

    if (middleFeedback.readMilliVolts() > FEEDBACK_THRESHOLD){
      servoController.middleBlocked = true;
    } else{
      servoController.middleBlocked = false;
    }

    if (ringFeedback.readMilliVolts() > FEEDBACK_THRESHOLD){
      servoController.ringBlocked = true;
    } else{
      servoController.ringBlocked = false;
    }

    if (pinkyFeedback.readMilliVolts() > FEEDBACK_THRESHOLD){
      servoController.pinkyBlocked = true;
    } else{
      servoController.pinkyBlocked = false;
    }
     // Delays the task for 100 ms (10 Hz)
    vTaskDelay(100 * portTICK_PERIOD_MS );
  }
}

/**
 * @brief Sums the current of all servo motors, blocks all servo motors if it 
 *        total current is greater than predefined MAX_SERVO_CURRENT
 * 
 * @param pvParameter Void Pointer 
 */
void chkMotorCurrent(void *pvParameter){
  while(1){
    //read current draw from all fingers
    currentSense.readAllFingerCurrents();

    // Compare total current to safety threshold
    if (currentSense.calculateTotalCurrent() > MAX_SERVO_CURRENT ){
      //Emergency Stop if current draw surpasses safety threshold
      servoController.stopAllMotion();
    }

    //THIS DELAY IS TEMPORARY FOR TESTING WILL BE REASSESSED MOVING FORWARD
    // Delays the task for 10 ms (100 Hz)
    vTaskDelay(10 * portTICK_PERIOD_MS );
  }
}

/**
 * @brief Sets up configures and initializes all necessary objects and tasks
 * 
 */
void setup() {
  Serial.begin(115200);

  // Set CPU clock to 80MHz
  setCpuFrequencyMhz(80); // Can be set to 80, 160, or 240 MHZ

  // Attach ADC object to GPIO pins
  myoware1.attach(8); // Myoware 1 is attached to GPIO 1
  myoware2.attach(7); // Myoware 2 is attached to GPIO 2

  // Attach Feedback sensors to GPIO pins
  thumbFeedback.attach(14);
  indexFeedback.attach(13);
  middleFeedback.attach(10);
  ringFeedback.attach(5);
  pinkyFeedback.attach(3);

  // Create RTOS Tasks 
  TaskHandle_t xHandle = NULL;
  
  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
      updateServoMotors,   // Function to be called
      "Update hand position",   // Name of task
      1024,         // Stack size (bytes in ESP32, words in FreeRTOS)
      NULL,         // Parameter to pass to function
      1,            // Task priority (0 to configMAX_PRIORITIES - 1)
      &xHandle,     // Task handle
      0);           // Run on core 1

  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
      readMyoSensor,        // Function to be called
      "Read Myoware Sensors",   // Name of task
      1024,         // Stack size (bytes in ESP32, words in FreeRTOS)
      NULL,         // Parameter to pass to function
      3,            // Task priority (0 to configMAX_PRIORITIES - 1)
      &xHandle,     // Task handle
      0);           // Run on core 1

  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
      chkHandCollision,     // Function to be called
      "Check Hand Collsion",   // Name of task
      1024,         // Stack size (bytes in ESP32, words in FreeRTOS)
      NULL,         // Parameter to pass to function
      2,            // Task priority (0 to configMAX_PRIORITIES - 1)
      &xHandle,     // Task handle
      0);           // Run on core 1

    xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
      chkMotorCurrent,     // Function to be called
      "Check motor current",   // Name of task
      1024,         // Stack size (bytes in ESP32, words in FreeRTOS)
      NULL,         // Parameter to pass to function
      2,            // Task priority (0 to configMAX_PRIORITIES - 1)
      &xHandle,     // Task handle
      0);           // Run on core 1
      
  //TODO: Create task for data processing   
}

void loop() {}