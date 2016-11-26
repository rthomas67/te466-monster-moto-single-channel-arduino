#include <Arduino.h>
#include "Te466Common.h"

/*
 * MotorState.h
 *
 *  Created on: Nov 25, 2016
 *      Author: rrt
 */

#ifndef LIBRARIES_TE466_DRIVER_MOTORSTATE_H_
#define LIBRARIES_TE466_DRIVER_MOTORSTATE_H_


class MotorState {

    int lastCurrentSensePinValue = 0;

    uint8_t currentSensePin;

    boolean overcurrentFaultDetected = false;

    uint8_t pinInA;
    uint8_t pinInB;
    uint8_t pinPWM;

    uint8_t currentStateIna = LOW;
    uint8_t currentStateInb = LOW;
    uint8_t currentStatePwm = 0;
    uint8_t desiredStateIna = LOW;
    uint8_t desiredStateInb = LOW;
    uint8_t desiredStatePwm = 0;

  public:
    MotorState(uint8_t pinInA, uint8_t pinInB, uint8_t pinPWM, uint8_t currentSensePin);

    void setDesiredState(uint8_t ina, uint8_t inb, uint8_t pwm);
    void updateMotorState();
    void stopMotor();
    boolean checkOvercurrent(int overcurrentThreshhold);
    boolean isOvercurrentFaultDetected();
    uint8_t getCurrentStateIna();
    uint8_t getCurrentStateInb();
    uint8_t getCurrentStatePwm();
    int getLastCurrentSensePinValue();

};



#endif /* LIBRARIES_TE466_DRIVER_MOTORSTATE_H_ */
