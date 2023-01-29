/*
 *  Prosthetic Arm Control (P.A.C) Current Sense
 *
 *  Created on: 06 Dec, 2022
 *      Author: Zachary Roberts (zroberts1@ycp.edu)
 * 
 *      This library is used to manage the ADC readings, conversions, and 
 *      calculations with the data from the Low End Current sense circuit for each
 *      respective finger on the prosthetic arm created in collaboration with 
 *      York College of Pennsylvania Capstone Course
 */

#include "PACCurrentSense.h"

PACCurrentSense::PACCurrentSense(){
    //Initializes the ADC pins to their respective fingers
    senseThumbCurrent.attach(12);
    senseIndexCurrent.attach(11);
    senseMiddleCurrent.attach(9);
    senseRingCurrent.attach(6);
    sensePinkyCurrent.attach(4);

    //initializes current readings to 0
    thumbCurrent = 0;
    indexCurrent = 0;
    middleCurrent = 0;
    ringCurrent = 0;
    pinkyCurrent = 0;
}

/**
 * @brief Sums the total current voltage readings and converts to current
 * 
 * @return uint16_t Total current consumed by all motors in mili-amperes
 */
uint16_t PACCurrentSense::calculateTotalCurrent() {
    //calculate total current in millivolts
    uint16_t totalVolts = thumbCurrent + indexCurrent + middleCurrent 
                            + ringCurrent + pinkyCurrent;

    //converts milivolts to milliamperes
    uint16_t totalCurrent = (totalVolts - 301.86) / 2.1;

    return totalCurrent;
}

/**
 * @brief Reads in the current state in voltage. Sets corresponding global
 *        variable to be accessed from main function
 * 
 */
void PACCurrentSense::readAllFingerCurrents(){
    thumbCurrent = senseCurrent(FINGER_THUMB_CHANNEL);
    indexCurrent = senseCurrent(FINGER_INDEX_CHANNEL);
    middleCurrent = senseCurrent(FINGER_MIDDLE_CHANNEL);
    ringCurrent = senseCurrent(FINGER_RING_CHANNEL);
    pinkyCurrent = senseCurrent(FINGER_PINKY_CHANNEL);
}

/**
 * @brief 
 * 
 * @param finger 
 * @return uint16_t 
 */
uint16_t PACCurrentSense::senseCurrent(uint8_t finger){
    switch(finger){
        case 0: // Thumb
            return senseThumbCurrent.readMilliVolts();
            break;
        case 1: // Index finger
            return senseIndexCurrent.readMilliVolts();
            break;
        case 2: // Middle finger
            return senseMiddleCurrent.readMilliVolts();
            break;
        case 3: // Ring finger
            return senseMiddleCurrent.readMilliVolts();
            break;
        case 4: // Pinky finger
            return sensePinkyCurrent.readMilliVolts();
            break;
    }

    // returns max 16 bit value if the request finger is undefined will block
    // all operations from occurring on the arm until resolved. 
    return 65535;
}
