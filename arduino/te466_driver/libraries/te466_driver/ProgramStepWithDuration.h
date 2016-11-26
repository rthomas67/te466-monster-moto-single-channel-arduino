#include "ProgramStep.h"
#include "Signal.h"

/*
 * ProgramStepWithDuration.h
 *
 *  Created on: Nov 25, 2016
 *      Author: rrt
 */

#ifndef LIBRARIES_TE466_DRIVER_PROGRAMSTEPWITHDURATION_H_
#define LIBRARIES_TE466_DRIVER_PROGRAMSTEPWITHDURATION_H_

class ProgramStepWithDuration: public ProgramStep {
    unsigned long durationMillis = 0;
    unsigned long startTime = 0;
    Signal *skipSignal = 0;
  public:
    ProgramStepWithDuration(uint8_t motorDirection, uint8_t motorSpeed, long durationMillis);

    ProgramStepWithDuration(uint8_t motorDirection, uint8_t motorSpeed, long durationMillis, Signal *skipSignal);

    boolean isDone();

    boolean activate(MotorState *motorState);

};

#endif /* LIBRARIES_TE466_DRIVER_PROGRAMSTEPWITHDURATION_H_ */
