#include "ProgramStepWithDuration.h"

/*
 * ProgramStepWithDuration.cpp
 *
 *      Author: rrt
 */

ProgramStepWithDuration::ProgramStepWithDuration(uint8_t motorDirection, uint8_t motorSpeed, long durationMillis) : ProgramStep(motorDirection, motorSpeed) {
    this->durationMillis = durationMillis;
}

ProgramStepWithDuration::ProgramStepWithDuration(uint8_t motorDirection, uint8_t motorSpeed, long durationMillis, Signal *skipSignal) : ProgramStepWithDuration(motorDirection, motorSpeed, durationMillis) {
    this->skipSignal = skipSignal;
}

boolean ProgramStepWithDuration::isDone() {
    /*
     * If this object was constructed with a Signal object, any time that signal
     * reports that it is "on", this step immediately reports that it is done.
     */
    if (skipSignal && skipSignal->isSignalActive()) {
        return true;
    }
    unsigned long endTime = startTime + durationMillis;
    return (millis() >= endTime);
}

boolean ProgramStepWithDuration::activate(MotorState *motorState) {
    /*
     * If this object was constructed with a Signal object, any time that signal
     * reports that it is "on", this step never gets activated.
     */
    if (skipSignal && skipSignal->isSignalActive()) {
        return false;
    }
    startTime = millis();  // reset the clock
    motorState->setDesiredState(desiredStateIna, desiredStateInb, motorSpeed);
    return true;
}

