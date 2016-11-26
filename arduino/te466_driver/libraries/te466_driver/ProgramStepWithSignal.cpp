#include "ProgramStepWithSignal.h"

/*
 * ProgramStepWithSignal.cpp
 *
 *  Created on: Nov 25, 2016
 *      Author: rrt
 */

ProgramStepWithSignal::ProgramStepWithSignal(uint8_t motorDirection, uint8_t motorSpeed, Signal *signalSource) : ProgramStep(motorDirection, motorSpeed) {
    this->signalSource = signalSource;
}

/*
 * Returns the false if isSignalActive() returns true.
 * In other words, the program step is _not_ done yet
 * as long as the signal _is_ active.
 */
boolean ProgramStepWithSignal::isDone() {
    return (signalSource->isSignalActive() != true);
}

boolean ProgramStepWithSignal::activate(MotorState *motorState) {
    if (signalSource && signalSource->isSignalActive()) {
        motorState->setDesiredState(desiredStateIna, desiredStateInb, motorSpeed);
        return true;
    }
    return false;
    // Otherwise don't do anything
}



