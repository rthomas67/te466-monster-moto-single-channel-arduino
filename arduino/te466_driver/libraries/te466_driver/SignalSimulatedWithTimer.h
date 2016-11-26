#include "Signal.h"

/*
 * SignalSimulatedWithTimer.h
 *
 *  Created on: Nov 25, 2016
 *      Author: rrt
 */

#ifndef LIBRARIES_TE466_DRIVER_SIGNALSIMULATEDWITHTIMER_H_
#define LIBRARIES_TE466_DRIVER_SIGNALSIMULATEDWITHTIMER_H_

class SignalSimulatedWithTimer : public Signal {
    unsigned long startTime;
  public:
    boolean isSignalActive();
    void startTimer();
};

#endif /* LIBRARIES_TE466_DRIVER_SIGNALSIMULATEDWITHTIMER_H_ */
