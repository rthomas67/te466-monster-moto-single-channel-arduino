#include "ProgramStep.h"
/*
 * ProgramStep.cpp
 *
 *  Created on: Nov 25, 2016
 *      Author: rrt
 */

ProgramStep::ProgramStep(uint8_t motorDirection, uint8_t motorSpeed) {
    this->motorDirection = motorDirection;
    this->motorSpeed = motorSpeed;
    if (motorDirection == FORWARD || motorDirection == BRAKEVCC)
        desiredStateIna = HIGH;
    else
        desiredStateIna= LOW;

    if ((motorDirection == BRAKEVCC) || (motorDirection == REVERSE))
        desiredStateInb = HIGH;
    else
        desiredStateInb = LOW;
}
