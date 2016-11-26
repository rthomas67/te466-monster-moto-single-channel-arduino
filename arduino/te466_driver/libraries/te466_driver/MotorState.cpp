#include "MotorState.h"
/*
 * MotorState.cpp
 *
 *  Created on: Nov 25, 2016
 *      Author: rrt
 */

MotorState::MotorState(uint8_t pinInA, uint8_t pinInB, uint8_t pinPWM, uint8_t currentSensePin) {
    this->pinInA = pinInA;
    this->pinInB = pinInB;
    this->pinPWM = pinPWM;
    this->currentSensePin = currentSensePin;
}

void MotorState::setDesiredState(uint8_t ina, uint8_t inb, uint8_t pwm) {
    desiredStateIna = ina;
    desiredStateInb = inb;
    desiredStatePwm = pwm;
}

void MotorState::updateMotorState() {
    if (desiredStateIna != currentStateIna) {
        currentStateIna = desiredStateIna;
        digitalWrite(pinInA, currentStateIna);
    }
    if (desiredStateInb != currentStateInb) {
        currentStateInb = desiredStateInb;
        digitalWrite(pinInB, currentStateInb);
    }
    if (desiredStatePwm != currentStatePwm) {
        currentStatePwm = desiredStatePwm;
        analogWrite(pinPWM, currentStatePwm);
    }
}

void MotorState::stopMotor() {
    setDesiredState(LOW, LOW, BRAKEGND);
    updateMotorState();
}

boolean MotorState::checkOvercurrent(int overcurrentThreshhold) {
    lastCurrentSensePinValue = analogRead(currentSensePin);
    if (lastCurrentSensePinValue > overcurrentThreshhold) {
        stopMotor();
        overcurrentFaultDetected = true;
        return true;
    } else {
        return false;
    }
}

boolean MotorState::isOvercurrentFaultDetected() {
    return overcurrentFaultDetected;
}

uint8_t MotorState::getCurrentStateIna() {
    return currentStateIna;
}

uint8_t MotorState::getCurrentStateInb() {
    return currentStateInb;
}

uint8_t MotorState::getCurrentStatePwm() {
    return currentStatePwm;
}

int MotorState::getLastCurrentSensePinValue() {
    return lastCurrentSensePinValue;
}
