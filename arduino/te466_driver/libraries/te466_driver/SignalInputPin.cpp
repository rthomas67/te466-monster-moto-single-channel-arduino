#include "SignalInputPin.h"
/*
 * SignalInputPin.cpp
 *
 *  Created on: Nov 25, 2016
 *      Author: rrt
 */

SignalInputPin::SignalInputPin(uint8_t digitalPinNumber) : Signal() {
    this->digitalPinNumber = digitalPinNumber;
}

SignalInputPin::SignalInputPin(uint8_t digitalPinNumber, uint8_t signalOnValue) : SignalInputPin(digitalPinNumber) {
    this->signalOnValue = signalOnValue;
}

/*
 * Implemented to report true when the current digital state of the pin
 * matches the value of signalOnValue.
 */
boolean SignalInputPin::isSignalActive() {
    uint8_t currentPinStatus = digitalRead(digitalPinNumber);
    if (wasActiveOnPreviousCall && !currentPinStatus) {
        wasActiveOnPreviousCall = false;
        startExtraTime();
    }
    if (currentPinStatus) {
        wasActiveOnPreviousCall = true;
    }
    return (isDuringExtraTime() || currentPinStatus == signalOnValue);
}

