#include "Signal.h"
/*
 * Signal.cpp
 *
 *  Created on: Nov 25, 2016
 *      Author: rrt
 */

Signal::Signal() {

}

void Signal::startExtraTime() {
    extraTimeStartTime = millis();
}

boolean Signal::isDuringExtraTime() {
    return (millis() <= (extraTimeStartTime + extraTimeDuration));
}

void Signal::setExtraTimeDuration(uint8_t extraTimeDuration) {
    this->extraTimeDuration = extraTimeDuration;
}

