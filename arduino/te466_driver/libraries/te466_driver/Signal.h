#include <Arduino.h>
/*
 * Signal.h
 *
 *  Created on: Nov 25, 2016
 *      Author: rrt
 */

#ifndef LIBRARIES_TE466_DRIVER_SIGNAL_H_
#define LIBRARIES_TE466_DRIVER_SIGNAL_H_

/*
 * Base Signal class includes support for subclasses to extend the time
 * they report active by a number of milliseconds set in the extraTimeDuration
 * member variable (by calling setExtraTimeDuration()).  This provides a way
 * to "soften" the reaction time when a signal is detected, which is sometimes
 * called "coasting".  Subclasses should call startExtraTime immediately after
 * detecting a signal condition to reset the extra-time clock, and thereafter,
 * delay reporting isSignalActive as false until isExtraTimeDone() returns true.
 */
class Signal {
    unsigned long extraTimeDuration = 0;
    unsigned long extraTimeStartTime = 0;
  protected:
    void startExtraTime();
    boolean isDuringExtraTime();
  public:
    Signal();
    virtual ~Signal() {

    }
    void setExtraTimeDuration(uint8_t extraTimeDuration);
    virtual boolean isSignalActive() = 0;
};

#endif /* LIBRARIES_TE466_DRIVER_SIGNAL_H_ */
