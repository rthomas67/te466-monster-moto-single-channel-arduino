#include <Arduino.h>
#include "Signal.h"

/*
 * SignalParkMotorFeedback.h
 *
 *  Created on: Nov 25, 2016
 *      Author: rrt
 */

#ifndef LIBRARIES_TE466_DRIVER_SIGNALPARKMOTORFEEDBACK_H_
#define LIBRARIES_TE466_DRIVER_SIGNALPARKMOTORFEEDBACK_H_

/*
 * This implementation of Signal reads logic pins which, together, indicate that
 * that the park function has been triggered and that the motor is not yet
 * in the park position.
 *
 * A program step using this Signal would continue at its speed and
 * direction until the motor is parked, so it should be followed by
 * a step that does whatever should follow parking the motor.
 *     Example 1: Follow with a step that remains stopped while just the trigger pin is still "on".
 *     Example 2: Continue with a sequence of motions meant to begin at the park position.
 *
 * This is designed to be used with windshield wiper motors
 * that have an internal switch that connects two contacts
 * when the crank-arm has reached a particular spot.
 *
 * Reminder: Be sure the input to the parkStatus digital pin is
 * properly isolated and noise filtered if it is anywhere near
 * the brushes or other electrical noise in the motor.
 */
class SignalParkMotorFeedback : public Signal {
    uint8_t triggerDigitalPinNumber;
    uint8_t parkStatusDigitalPinNumber;
    /*
     * Note: The pins should be initialized as INPUT_PULLUP (open == "off" == HIGH)
     * unless the default "on" values are overridden upon object construction.
     */
    uint8_t triggerDigitalPinOn = LOW;
    uint8_t parkStatusDigitalPinOn = LOW;
  public:
    SignalParkMotorFeedback(uint8_t triggerDigitalPinNumber, uint8_t parkStatusDigitalPinNumber);
    SignalParkMotorFeedback(uint8_t triggerDigitalPinNumber, uint8_t parkStatusDigitalPinNumber, uint8_t triggerDigitalPinOn, uint8_t parkStatusDigitalPinOn);
    /*
     * This Signal reports that it is "active" as long as the triggering pin
     * is "on" and the park position pin is still "off" (i.e. not parked yet.)
     */
    boolean isSignalActive();

};

#endif /* LIBRARIES_TE466_DRIVER_SIGNALPARKMOTORFEEDBACK_H_ */
