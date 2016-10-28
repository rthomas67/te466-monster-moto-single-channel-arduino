#include "te466_driver.h"

MotorState::MotorState(uint8_t pinInA, uint8_t pinInB, uint8_t pinPWM) {
    this->pinInA = pinInA;
    this->pinInB = pinInB;
    this->pinPWM = pinPWM;
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
        motor1State->stopMotor();
        overcurrentFaultDetected = true;
        return true;
	} else {
	    return false;
	}
}

boolean MotorState::isOvercurrentFaultDetected() {
	return overcurrentFaultDetected;
}

int MotorState::getLastCurrentSensePinValue() {
	return lastCurrentSensePinValue;
}

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
    
Signal::Signal() {

}

void Signal::startExtraTime() {
	extraTimeStartTime = millis();
}

boolean Signal::isDuringExtraTime() {
	return (millis() <= (extraTimeStartTime + extraTimeDuration));
}

void Signal::setExtraTimeDuration(uint8_t extraTimeDuration) {
	this->extraTimeDuration = extraTimeDuration;
}

ProgramStepWithDuration::ProgramStepWithDuration(uint8_t motorDirection, uint8_t motorSpeed, long durationMillis) : ProgramStep(motorDirection, motorSpeed) {
	this->durationMillis = durationMillis;
}
    
ProgramStepWithDuration::ProgramStepWithDuration(uint8_t motorDirection, uint8_t motorSpeed, long durationMillis, Signal *skipSignal) : ProgramStepWithDuration(motorDirection, motorSpeed, durationMillis) {
	this->skipSignal = skipSignal;
}
    
boolean ProgramStepWithDuration::isDone() {
	/*
	 * If this object was constructed with a Signal object, any time that signal
	 * reports that it is "on", this step immediately reports that it is done.
	 */
	if (skipSignal && skipSignal->isSignalActive()) {
		return true;
	}
	unsigned long endTime = startTime + durationMillis;
	return (millis() >= endTime);
}

boolean ProgramStepWithDuration::activate(MotorState *motorState) {
	/*
	 * If this object was constructed with a Signal object, any time that signal
	 * reports that it is "on", this step never gets activated.
	 */
	if (skipSignal && skipSignal->isSignalActive()) {
		return false;
	}
	startTime = millis();  // reset the clock
	motorState->setDesiredState(desiredStateIna, desiredStateInb, motorSpeed);
	return true;
}

boolean SignalSimulatedWithTimer::isSignalActive() {
	return (millis() > (startTime + 3000));
}

void SignalSimulatedWithTimer::startTimer() {
	startTime = millis();
}

SignalInputPin::SignalInputPin(uint8_t digitalPinNumber) : Signal() {
	this->digitalPinNumber = digitalPinNumber;
}

SignalInputPin::SignalInputPin(uint8_t digitalPinNumber, uint8_t signalOnValue) : SignalInputPin(digitalPinNumber) {
	this->signalOnValue = signalOnValue;
}

/*
 * Implemented to report true when the current digital state of the pin
 * matches the value of signalOnValue.
 */
boolean SignalInputPin::isSignalActive() {
	uint8_t currentPinStatus = digitalRead(digitalPinNumber);
	if (wasActiveOnPreviousCall && !currentPinStatus) {
		wasActiveOnPreviousCall = false;
		startExtraTime();
	}
	if (currentPinStatus) {
		wasActiveOnPreviousCall = true;
	}
	return (isDuringExtraTime() || currentPinStatus == signalOnValue);
}

SignalParkMotorFeedback::SignalParkMotorFeedback(uint8_t triggerDigitalPinNumber, uint8_t parkStatusDigitalPinNumber) : Signal() {
	this->triggerDigitalPinNumber = triggerDigitalPinNumber;
	this->parkStatusDigitalPinNumber = parkStatusDigitalPinNumber;
}

SignalParkMotorFeedback::SignalParkMotorFeedback(uint8_t triggerDigitalPinNumber, uint8_t parkStatusDigitalPinNumber, uint8_t triggerDigitalPinOn, uint8_t parkStatusDigitalPinOn)
		: SignalParkMotorFeedback(triggerDigitalPinNumber, parkStatusDigitalPinNumber) {
	this->triggerDigitalPinOn = triggerDigitalPinOn;
	this->parkStatusDigitalPinOn = parkStatusDigitalPinOn;
}

/*
 * This Signal reports that it is "active" as long as the triggering pin
 * is "on" and the park position pin is still "off" (i.e. not parked yet.)
 */
boolean SignalParkMotorFeedback::isSignalActive() {
	return (digitalRead(triggerDigitalPinNumber) == triggerDigitalPinOn
		&& digitalRead(parkStatusDigitalPinNumber) != parkStatusDigitalPinOn);
}

ProgramStepWithSignal::ProgramStepWithSignal(uint8_t motorDirection, uint8_t motorSpeed, Signal *signalSource) : ProgramStep(motorDirection, motorSpeed) {
	this->signalSource = signalSource;
}

/*
 * Returns the false if isSignalActive() returns true.
 * In other words, the program step is _not_ done yet
 * as long as the signal _is_ active.
 */
boolean ProgramStepWithSignal::isDone() {
	return (signalSource->isSignalActive() != true);
}

boolean ProgramStepWithSignal::activate(MotorState *motorState) {
	if (signalSource && signalSource->isSignalActive()) {
		motorState->setDesiredState(desiredStateIna, desiredStateInb, motorSpeed);
		return true;
	}
	return false;
	// Otherwise don't do anything
}

Program::Program() {
}

void Program::addProgramStep(ProgramStep *programStep) {
	// Create or expand the array of program steps
	ProgramStep** programStepsOld = programSteps;
	programSteps = new ProgramStep*[programStepCount + 1];
	// copy pointers from the old array to the expanded array
	if (programStepsOld != 0) {
		for (int i=0; i<programStepCount; i++) {
			programSteps[i] = programStepsOld[i];
		}
	}
	delete programStepsOld;
	// array index is 0-based but the array is one larger now, so
	programSteps[programStepCount] = programStep;
	// increase this for next time
	programStepCount++;
}

uint8_t Program::getProgramStepCount() {
	return programStepCount;
}

/*
 * The main loop calls this over and over just before calling updateMotorState.
 * This function continues advancing through steps until it finds one that
 * reports that it is not done.
 * Note: activate(MotorState) is called on each next step as it becomes the
 * current step before looping and asking it whether it is already done.
 * This allows activate() to reset default pin conditions, timer start values, etc.
 * before isDone() checks to see if the step should be skipped immediately.
 * Note: Since this keeps looking when a step says it is done, there can
 * be several steps in a program that only execute in certain conditions.
 */
void Program::updateProgramState(MotorState *motorState) {
	/*
	 * Keep skipping through steps until one says that it is not done
	 * after it has been activated.
	 */
	while (programSteps[currentProgramStepIndex]->isDone()) {
		currentProgramStepIndex++;
		if (currentProgramStepIndex >= programStepCount) {
			currentProgramStepIndex = 0;  // loop and repeat program
		}
		programSteps[currentProgramStepIndex]->activate(motorState);
	}
}

