#include "SignalSimulatedWithTimer.h"
/*
 * SignalSimulatedWithTimer.cpp
 *
 *  Created on: Nov 25, 2016
 *      Author: rrt
 */

boolean SignalSimulatedWithTimer::isSignalActive() {
    return (millis() > (startTime + 3000));
}

void SignalSimulatedWithTimer::startTimer() {
    startTime = millis();
}


