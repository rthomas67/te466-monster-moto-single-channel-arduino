#include <Arduino.h>
#include "Te466Common.h"
#include "MotorState.h"
/*
 * ProgramStep.h
 *
 *  Created on: Nov 25, 2016
 *      Author: rrt
 */

#ifndef LIBRARIES_TE466_DRIVER_PROGRAMSTEP_H_
#define LIBRARIES_TE466_DRIVER_PROGRAMSTEP_H_


class ProgramStep {
    uint8_t motorDirection = BRAKEGND;

    // derived upon construction
  protected:
    uint8_t motorSpeed = 0;
    uint8_t desiredStateIna;
    uint8_t desiredStateInb;

  public:
    /**
     * If durationMillis == 0, this program step will only report false
     * from isDurationComplete() when  isSignalActive is true.
     */
    ProgramStep(uint8_t motorDirection, uint8_t motorSpeed);
    virtual ~ProgramStep() {
        // TODO: Implement default destructor
    }

    virtual boolean activate(MotorState *motorState) = 0;

    virtual boolean isDone(void) = 0;

};



#endif /* LIBRARIES_TE466_DRIVER_PROGRAMSTEP_H_ */
