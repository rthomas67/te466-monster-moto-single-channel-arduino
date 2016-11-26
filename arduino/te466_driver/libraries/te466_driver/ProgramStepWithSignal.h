#include "ProgramStep.h"
#include "Signal.h"

/*
 * ProgramStepWithSignal.h
 *
 *  Created on: Nov 25, 2016
 *      Author: rrt
 */

#ifndef LIBRARIES_TE466_DRIVER_PROGRAMSTEPWITHSIGNAL_H_
#define LIBRARIES_TE466_DRIVER_PROGRAMSTEPWITHSIGNAL_H_

/*
 * Note: This can be used as a "pause until signal is not active"
 * by simply never calling activate, or it may follow another
 * signal step with the same signal but different motor settings.
 */
class ProgramStepWithSignal: public ProgramStep {
  protected:
    Signal *signalSource = 0;

  public:
    ProgramStepWithSignal(uint8_t motorDirection, uint8_t motorSpeed, Signal *signalSource);

    /*
     * Returns the false if isSignalActive() returns true.
     * In other words, the program step is _not_ done yet
     * as long as the signal _is_ active.
     */
    boolean isDone();
    boolean activate(MotorState *motorState);
};

#endif /* LIBRARIES_TE466_DRIVER_PROGRAMSTEPWITHSIGNAL_H_ */
