#include "SignalParkMotorFeedback.h"
/*
 * SignalParkMotorFeedback.cpp
 *
 *  Created on: Nov 25, 2016
 *      Author: rrt
 */

SignalParkMotorFeedback::SignalParkMotorFeedback(uint8_t triggerDigitalPinNumber, uint8_t parkStatusDigitalPinNumber) : Signal() {
    this->triggerDigitalPinNumber = triggerDigitalPinNumber;
    this->parkStatusDigitalPinNumber = parkStatusDigitalPinNumber;
}

SignalParkMotorFeedback::SignalParkMotorFeedback(uint8_t triggerDigitalPinNumber, uint8_t parkStatusDigitalPinNumber, uint8_t triggerDigitalPinOn, uint8_t parkStatusDigitalPinOn)
        : SignalParkMotorFeedback(triggerDigitalPinNumber, parkStatusDigitalPinNumber) {
    this->triggerDigitalPinOn = triggerDigitalPinOn;
    this->parkStatusDigitalPinOn = parkStatusDigitalPinOn;
}

/*
 * This Signal reports that it is "active" as long as the triggering pin
 * is "on" and the park position pin is still "off" (i.e. not parked yet.)
 */
boolean SignalParkMotorFeedback::isSignalActive() {
    return (digitalRead(triggerDigitalPinNumber) == triggerDigitalPinOn
        && digitalRead(parkStatusDigitalPinNumber) != parkStatusDigitalPinOn);
}




