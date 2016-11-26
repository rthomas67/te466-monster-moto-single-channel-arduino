#include <Arduino.h>
#include "Signal.h"

/*
 * SignalInputPin.h
 *
 *  Created on: Nov 25, 2016
 *      Author: rrt
 */

#ifndef LIBRARIES_TE466_DRIVER_SIGNALINPUTPIN_H_
#define LIBRARIES_TE466_DRIVER_SIGNALINPUTPIN_H_

class SignalInputPin : public Signal {
    /*
     * The pin from which the digital state is read and compared with the signalOnValue
     */
    uint8_t digitalPinNumber;
    /* Th signalOnValue variable holds HIGH or LOW, depending upon which of those represents the "signal-on" state
     * The default is LOW, which assumes the pin is initialized with INPUT_PULLUP (i.e. open == HIGH == "off")
     * If this object is constructed with a custom signalOnValue (which would be superfluous unless it is HIGH),
     *     then isSignalActive() will return "true":
     *         1) UNLESS the pin is connected to ground,
     *         or
     *         2) IF the pin is initialized as a normal INPUT (without using the built in PULLUP resistor).
     *
     * Note: If a digital pin is used without the internal pullup, it should use
     *    an external pull-down resistor (~10K)
     */
    uint8_t signalOnValue = LOW;
    boolean wasActiveOnPreviousCall = false;
  public:
    SignalInputPin(uint8_t digitalPinNumber);
    SignalInputPin(uint8_t digitalPinNumber, uint8_t signalOnValue);
    /*
     * Implemented to report true when the current digital state of the pin
     * matches the value of signalOnValue.
     */
    boolean isSignalActive();

};

#endif /* LIBRARIES_TE466_DRIVER_SIGNALINPUTPIN_H_ */
