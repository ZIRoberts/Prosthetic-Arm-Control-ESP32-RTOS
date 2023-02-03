#pragma once

#include <../lib/ESP32AnalogRead/ESP32AnalogRead.h>
#include <Arduino.h>

//Defines Constant for PWM Channels
//Used to identify which finger an object is to operate on
#define FINGER_THUMB_CHANNEL 0
#define FINGER_INDEX_CHANNEL 1
#define FINGER_MIDDLE_CHANNEL 2
#define FINGER_RING_CHANNEL 3
#define FINGER_PINKY_CHANNEL 4


class PACFSRFeedback {
private :
    //ADC Objects
    ESP32AnalogRead indexFeedback;
    ESP32AnalogRead middleFeedback;
    ESP32AnalogRead thumbFeedback;
    ESP32AnalogRead ringFeedback;
    ESP32AnalogRead pinkyFeedback;

    //
    uint16_t thumbFeedbackRaw;
    uint16_t indexFeedbackRaw;
    uint16_t middleFeedbackRaw;
    uint16_t ringFeedbackRaw;
    uint16_t pinkyFeedbackRaw;

public :    
    // Constructor
    PACFSRFeedback();

    //
    uint16_t thumbFeedbackGrams;
    uint16_t indexFeedbackGrams;
    uint16_t middleFeedbackGrams;
    uint16_t ringFeedbackGrams;
    uint16_t pinkyFeedbackGrams;

    void readAllFingerFeedback();
    void calculateFeedbackGrams();
    uint16_t senseFeedback(uint8_t finger);
};

