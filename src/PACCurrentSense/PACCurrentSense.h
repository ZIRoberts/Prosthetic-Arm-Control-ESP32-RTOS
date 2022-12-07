#pragma once

#include <../lib/ESP32AnalogRead/ESP32AnalogRead.h>

//Defines Constant for PWM Channels
//Used to identify which finger an object is to operate on
#define FINGER_THUMB_CHANNEL 0
#define FINGER_INDEX_CHANNEL 1
#define FINGER_MIDDLE_CHANNEL 2
#define FINGER_RING_CHANNEL 3
#define FINGER_PINKY_CHANNEL 4


class PACCurrentSense {
private :
    //ADC Objects
    static ESP32AnalogRead senseThumbCurrent; 
    static ESP32AnalogRead senseIndexCurrent;
    static ESP32AnalogRead senseMiddleCurrent;
    static ESP32AnalogRead senseRingCurrent;
    static ESP32AnalogRead sensePinkyCurrent;

    //stores ADC reading current sensor
    uint16_t thumbCurrent;
    uint16_t indexCurrent;
    uint16_t middleCurrent;
    uint16_t ringCurrent;
    uint16_t pinkyCurrent;

public :    
    // Constructor
    PACCurrentSense();

    void readAllFingerCurrents();
    uint16_t calculateTotalCurrent();
    uint16_t senseCurrent(uint8_t finger);
};

