/*
 *  Prosthetic Arm Control (P.A.C) ESP32S2 RTOS
 *
 *  Created on: 24 Nov, 2022
 *      Author: Zachary Roberts
 *
 *      This program is used to control the Prosthetic Arm created in
 *      collaboration with York College of Pennsylvania Capstone course.
 */

#include <../lib/sparthan_myo/myo.h>
#include <Arduino.h>
#include <PACFSRFeedback/PACFSRFeedback.h>  // FSR Feedback library for prosethic arm
#include <PACServoDriver/PACServoDriver.h>  // Motor control libary for prosehtic arm
#include <SPICurrentSense/SPICurrentSense.h>
#include <freertos/queue.h>

#include <cstdlib>
#include <deque>  // C++ libraries are not native to arduino, included within ESPIDF

#include "config.h"  // config file with common definitions

// Creates Servo Driver, Current Sense, and Feedback object
static PACServoDriver servoController;
static PACFSRFeedback fsrFeedback;
static SPICurrentSense spiCurrentSense;

// Creates Myo Armband object
static armband myo;

// Buffer defintion for EMG Sensor
static std::deque<buffer> myoBuffer;
// Queue to pass buffer data between
QueueHandle_t xQueue;

// Used to pass hand position from signal processing task to update servos task
uint8_t handPosition = 0;

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
    switch (handPosition) {
      case 0:
        servoController.openHand();
        break;
      case 1:
        servoController.largeDiameter();
        break;
      case 2:
        servoController.indexFingerPointing();
        break;
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
void transferMyoData(void *pvParameter) {
  Serial.println("transferMyoData: ");
  buffer tempBuffer;
  while (1) {
    // Checks if buffer is full and removes oldest data if necessary
    if (myoBuffer.size() >= MAX_BUFFER_SIZE) {
      myoBuffer.pop_front();
    }

    // Checks if new data is available an updates buffer
    if (xQueueReceive(xQueue, &tempBuffer, 0) == pdTRUE) {
      myoBuffer.push_back(tempBuffer);
    }

    // Checks if new data is available an updates buffer
    if (xQueueReceive(xQueue, &tempBuffer, 0) == pdTRUE) {
      myoBuffer.push_back(tempBuffer);
    }

    // Delays the task for 5 ms (200 Hz)
    vTaskDelay(5 * portTICK_PERIOD_MS);
  }
}

/**
 * @brief Runs on CORE 0. Is executed when armband notifies BLE characteristic
 * of a ready EMG data sample. Passes he EMG sample to CORE 1 for Processing.
 *
 * @param pBLERemoteCharacteristic Model of remote BLE characteristic
 * @param pData Raw EMG data received from myo armband
 * @param length leng of raw EMG data
 * @param isNotify Notification status of BLE characteristic
 */
void emg_callback(BLERemoteCharacteristic *pBLERemoteCharacteristic,
                  uint8_t *pData, size_t length, bool isNotify) {
  myohw_emg_data_t *emg_data = (myohw_emg_data_t *)pData;
  buffer tempBuffer1;
  buffer tempBuffer2;

  tempBuffer1 = {emg_data->sample1[0], emg_data->sample1[1],
                 emg_data->sample1[2], emg_data->sample1[3],
                 emg_data->sample1[4], emg_data->sample1[5],
                 emg_data->sample1[6], emg_data->sample1[7]};

  tempBuffer2 = {emg_data->sample2[0], emg_data->sample2[1],
                 emg_data->sample2[2], emg_data->sample2[3],
                 emg_data->sample2[4], emg_data->sample2[5],
                 emg_data->sample2[6], emg_data->sample2[7]};

  // Pushes data to inter-core buffer
  // If Queue is full for longer than 10 ms, clear the buffer and push new data
  if (xQueueSend(xQueue, &tempBuffer1, 10) == errQUEUE_FULL) {
    xQueueReset(xQueue);
    xQueueSend(xQueue, &tempBuffer1, 0);
  }

  if (xQueueSend(xQueue, &tempBuffer2, 10) == errQUEUE_FULL) {
    xQueueReset(xQueue);
    xQueueSend(xQueue, &tempBuffer2, 0);
  }
}

/**
 * @brief Connects to Myo armband via BLE and register for BLE notification
 *        events for data stream.
 *        This is the only task that runs on Core 0.
 *
 * @param pvParameter void pointer
 */
void readMyoArmband(void *pvParameter) {
  while (1) {
    // Connects to Myo Armband sensor
    if (!myo.connected) {
      Serial.println("Connecting...");
      myo.connect();
      Serial.println(" - Connected");
      delay(100);

      // Armband sleep mode
      // sleep mode 0 = turn off after period of inactivity
      // sleep mode 1 = remain on until battery is depleted
      myo.set_sleep_mode(1);

      // Set data tranmission mode
      myo.set_myo_mode(myohw_emg_mode_send_emg,          // Enables EMG data
                       myohw_imu_mode_none,              // Disable IMU Data
                       myohw_classifier_mode_disabled);  // Disable Classifier

      myo.emg_notification(TURN_ON)->registerForNotify(emg_callback);
    }

    delay(10);
  }
}

/**
 * @brief chkHandCollision: Checks if any of the fingers have collided with
 *        an object
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

    Serial.print("Index FSR: ");
    Serial.println(fsrFeedback.indexFeedbackRaw);
    Serial.println(" ");

    Serial.print("Middle FSR: ");
    Serial.println(fsrFeedback.middleFeedbackRaw);
    Serial.println(" ");

    Serial.print("Ring FSR: ");
    Serial.println(fsrFeedback.ringFeedbackRaw);
    Serial.println(" ");

    Serial.print("Pinky FSR: ");
    Serial.println(fsrFeedback.pinkyFeedbackRaw);
    Serial.println(" ");

    Serial.print("Thumb FSR: ");
    Serial.println(fsrFeedback.thumbFeedbackRaw);
    Serial.println(" ");

    Serial.print("Palm FSR: ");
    Serial.println(fsrFeedback.palmFeedbackRaw);
    Serial.println(" ");
    delay(250);

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
      } else if (fsrFeedback.palmFeedbackGrams <= (marginThreshold)) {
        hysteresis = false;
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
        } else if (fsrFeedback.palmFeedbackGrams >= (marginThreshold)) {
          hysteresis = true;
        }
      }

      // Delays the task for 100 ms (10 Hz)
      vTaskDelay(100 * portTICK_PERIOD_MS);
    }
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
    // currentSense.readAllFingerCurrents();

    spiCurrentSense.readAllFingerCurrents();

    // Serial.print("Thumb Current: ");
    // Serial.println(spiCurrentSense.thumbCurrent);

    // Compare total current to safety threshold
    if (spiCurrentSense.calculateTotalCurrent() > MAX_SERVO_CURRENT) {
      // Emergency Stop if current draw surpasses safety threshold
      servoController.stopAllMotion();
    }

    //  Delays the task for 10 ms (100 Hz)
    vTaskDelay(10 * portTICK_PERIOD_MS);
  }
}

/**
 * @brief Processes the signals received from the myo armband and determines
 * the corresponding hand position. Currently implemented using mean absolute
 *        value with a set hysteresis threshold. Will be changed in future
 *        versions to utilized Linear Discriminant Analysis
 *
 * @param pvParameter
 */
void signalProcessor(void *pvParameter) {
  while (1) {
    if (myoBuffer.size() == MAX_BUFFER_SIZE) {
      // Processes signal into mean absolute value for EMG channels 3 and 7
      uint32_t chnl_3_MAV = 0;
      uint32_t chnl_7_MAV = 0;
      uint8_t chnl_3_threshold = 40;
      // uint8_t chnl_7_threshold = 25;
      uint8_t chnl_7_threshold = 25;

      // Sums the absolute values of the channels and stores in respective
      // temporary buffer integers
      for (uint8_t sampleNUM = 0; sampleNUM < 50; sampleNUM++) {
        chnl_3_MAV += abs(myoBuffer[200 + sampleNUM].myo3);
        chnl_7_MAV += abs(myoBuffer[200 + sampleNUM].myo7);
      }

      // Averages the samples for 250 ms windows
      // chnl_3_MAV /= 45;
      // chnl_7_MAV /= 25;
      chnl_3_MAV /= 25;
      chnl_7_MAV /= 25;

      Serial.println("Hand Position: ");
      if (chnl_3_MAV >= chnl_3_threshold) {
        // set large diameter flag
        handPosition = 1;
      } else if (chnl_7_MAV >= chnl_7_threshold) {
        // set index finger pointing threshold
        handPosition = 2;
      } else {
        // set platform push threshold
        handPosition = 0;
      }
      Serial.println(handPosition);
    }

    //  Delays the task for 200 ms (5 Hz)
    vTaskDelay(20 * portTICK_PERIOD_MS);
  }
}

/**
 * @brief Runs once when the ESP32 microcontroller boots up.
 *        Calibrates each current sense circuit to ensure accuracy
 *        of each individual current sense circuit.
 *
 */
void calibrateCurrentSense() {
  // Sets all Servos to calibration state
  servoController.calibration();

  // Creates temporary variables for calibration calculations
  uint16_t numCalibrationSamples = 250;
  uint32_t thumbCalibration = 0;
  uint32_t indexCalibration = 0;
  uint32_t middleCalibration = 0;
  uint32_t ringCalibration = 0;
  uint32_t pinkyCalibration = 0;

  // Reads in 500 current sense values
  for (uint16_t i = 0; i < numCalibrationSamples; i++) {
    spiCurrentSense.readAllFingerCurrents();

    thumbCalibration += spiCurrentSense.thumbCurrent;
    indexCalibration += spiCurrentSense.indexCurrent;
    middleCalibration += spiCurrentSense.middleCurrent;
    ringCalibration += spiCurrentSense.ringCurrent;
    middleCalibration += spiCurrentSense.middleCurrent;
    pinkyCalibration += spiCurrentSense.pinkyCurrent;

    // Short delay to ensure current average is taken over time
    // delay is used instead of vTaskDelay to ensure no tasks that could
    // change motor states are running
    delay(1);
  }

  // Average base currents are calculated
  thumbCalibration /= numCalibrationSamples;
  indexCalibration /= numCalibrationSamples;
  middleCalibration /= numCalibrationSamples;
  ringCalibration /= numCalibrationSamples;
  pinkyCalibration /= numCalibrationSamples;

  // Sets calculated offsets to corresponding variables
  spiCurrentSense.setCalibrationOffsets(thumbCalibration, indexCalibration,
                                        middleCalibration, ringCalibration,
                                        pinkyCalibration);
  spiCurrentSense.calibrated = true;

  // Sets hand to default position
  servoController.openHand();
}

/**
 * @brief Sets up, configures, and initializes all necessary objects and tasks
 *
 */
void setup() {
  // Starts serial communication at 115200 baud rate
  Serial.begin(115200);

  // Set CPU clock to 80MHz
  setCpuFrequencyMhz(80);  // Can be set to 80, 160, or 240 MHZ
  esp_log_level_set("*", ESP_LOG_DEBUG);

  calibrateCurrentSense();

  xQueue = xQueueCreate(10, sizeof(buffer));

  // Create RTOS Tasks
  TaskHandle_t xHandle = NULL;

  xTaskCreatePinnedToCore(     // Use xTaskCreate() in vanilla FreeRTOS
      updateServoMotors,       // Function to be called
      "Update hand position",  // Name of task
      2024,                    // Stack size (bytes in ESP32, words in FreeRTOS)
      NULL,                    // Parameter to pass to function
      1,                       // Task priority (0 to configMAX_PRIORITIES - 1)
      &xHandle,                // Task handle
      1);                      // Run on core 1

  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
      transferMyoData,      // Function to be called
      "Transfer EMG Data",  // Name of task
      2048,                 // Stack size (bytes in ESP32, words in FreeRTOS)
      NULL,                 // Parameter to pass to function
      2,                    // Task priority (0 to configMAX_PRIORITIES - 1)
      &xHandle,             // Task handle
      1);                   // Run on core 1

  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
      readMyoArmband,       // Function to be called
      "Read EMG armband",   // Name of task
      10240,                // Stack size (bytes in ESP32, words in FreeRTOS)
      NULL,                 // Parameter to pass to function
      3,                    // Task priority (0 to configMAX_PRIORITIES - 1)
      &xHandle,             // Task handle
      0);                   // Run on core 0

  xTaskCreatePinnedToCore(    // Use xTaskCreate() in vanilla FreeRTOS
      chkHandCollision,       // Function to be called
      "Check Hand Collsion",  // Name of task
      2048,                   // Stack size (bytes in ESP32, words in FreeRTOS)
      NULL,                   // Parameter to pass to function
      2,                      // Task priority (0 to configMAX_PRIORITIES - 1)
      &xHandle,               // Task handle
      1);                     // Run on core 1

  xTaskCreatePinnedToCore(    // Use xTaskCreate() in vanilla FreeRTOS
      chkMotorCurrent,        // Function to be called
      "Check motor current",  // Name of task
      2048,                   // Stack size (bytes in ESP32, words in FreeRTOS)
      NULL,                   // Parameter to pass to function
      2,                      // Task priority (0 to configMAX_PRIORITIES - 1)
      &xHandle,               // Task handle
      1);                     // Run on core 1

  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
      signalProcessor,      // Function to be called
      "Transfer EMG Data",  // Name of task
      2048,                 // Stack size (bytes in ESP32, words in FreeRTOS)
      NULL,                 // Parameter to pass to function
      3,                    // Task priority (0 to configMAX_PRIORITIES - 1)
      &xHandle,             // Task handle
      1);                   // Run on core 1
}

void loop() {}
