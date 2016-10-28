#include "Arduino.h"
#include "te466_driver.h"

#define CS_THRESHHOLD 800

static const uint8_t PIN_INA = 5;
static const uint8_t PIN_INB = 6;
static const uint8_t PIN_PWM = 9;
static const uint8_t PIN_CS = A3;
static const uint8_t PIN_EN = A6;

MotorState *motorState;
Program *program;

void setup()
{
    pinMode(PIN_INA, OUTPUT);
    pinMode(PIN_INB, OUTPUT);
    pinMode(PIN_PWM, OUTPUT);
    pinMode(PIN_EN, INPUT);
    pinMode(PIN_CS, INPUT);

    motorState = new MotorState(PIN_INA, PIN_INB, PIN_PWM, PIN_CS);
    program = new Program();
    program->addProgramStep(new ProgramStepWithDuration(FORWARD, 127, 1000));
    program->addProgramStep(new ProgramStepWithDuration(REVERSE, 127, 1000));
    program->addProgramStep(new ProgramStepWithDuration(BRAKEGND, 0, 1000));
}


void loop()
{
    if (motorState->isOvercurrentFaultDetected()) {
        // fast-blink the on-board LED
        digitalWrite(13, HIGH);
        delay(50);
        digitalWrite(13, LOW);
        delay(50);
    } else {
        // Check every loop to be sure the motor isn't stalled / over-current
    	if (!motorState->checkOvercurrent(CS_THRESHHOLD)) {
            program->updateProgramState(motorState);
            motorState->updateMotorState();
        }
    }

}
