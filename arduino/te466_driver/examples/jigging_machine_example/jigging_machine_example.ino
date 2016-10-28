#include "te466_driver.h"
/*
This code and the corresponding pin choices are intended for Arduino NANO v3.0

 Date: 2016/10/10
 Author: R Thomas

 Reminder: digitalWrite(someAnalogPin, HIGH) turns on the built-in pull-up resistor on an analog pin
 Reminder: analogWrite(someDigitalPin, value) enables PWM output on a digital pin (if capable)
 */

#define CS_THRESHOLD 600
#define UPDATE_INTERVAL 10


/*  TE466 VNH2SP30 pin definitions */
// INA: digital/logic-level output HIGH for FORWARD, LOW for REVERSE
// direct connection to TE466 - INA
// Arduino NANO pin D5
static const uint8_t PIN_INA = 5;

// INB: digital/logic-level output LOW for FORWARD, HIGH for REVERSE
// direct connection to TE466 - INB
// Arduino NANO pin D6
static const uint8_t PIN_INB = 6;

// PWM: modulated digital/logic-level output to regulate duty-cycle on output
// direct connection to TE466 - PWM (sometimes labeled PMW)
// Arduino NANO pin D9
static const uint8_t PIN_PWM = 9;

// CS: analog input for "current sense"
// direct connection to TE466 - CS
// Arduino NANO pin A3
static const uint8_t PIN_CS = A3;

// EN: "ENable" status of switches output (Analog pin)
// direct connection to TE466 - EN
// Arduino NANO pin A6
static const uint8_t PIN_EN = A6;

// PARK input trigger digital pin
// When this pin goes low it signals the motor to run forward until it is in the parked position
static const uint8_t PIN_PARK_TRIGGER = 3;

// WIRING REQUIREMENT 1: This input must be directly wired to a switch inside the motor that feeds
// back the state of OUTA until the motor reaches the park position.
// WIRING REQUIREMENT 2: This input must be externally pulled via 10K resistor to the current state of OUTB
// The motor is in the process of parking while:
// PIN_PARK_FEEDBACK is in the opposite state of OUTB
// TODO: Determine if this requires opto-isolation
static const uint8_t PIN_PARK_FEEDBACK = 4;

static const uint8_t PIN_PROGRAM_SWITCH_1 = 7;
static const uint8_t PIN_PROGRAM_SWITCH_2 = 8;

// On board LED
static const uint8_t PIN_STATUS = 13;

boolean motorCurrentFaultDetected = false;

MotorState *motor1State;

// Note: Pins are initialized with pullup enabled.  Boolean algebra is backwards here.
// index [0][0] - both PROGRAM_SWITCH pins == HIGH
// index [0][1] - PROGRAM_SWITCH pins 1,2 == HIGH,LOW
// index [1][0] - PROGRAM_SWITCH pins 1,2 == LOW,HIGH
// index [1][0] - both PROGRAM_SWITCH pins == LOW
Program *programs[2][2];

void setup()
{

    pinMode(PIN_STATUS, OUTPUT);
    // Init motor controller (digital and PWM) pins as outputs
    pinMode(PIN_INA, OUTPUT);
    pinMode(PIN_INB, OUTPUT);
    pinMode(PIN_PWM, OUTPUT);  // analogWrite will set PWM duty cycle
    // Init "enable" and "current sense" pins as inputs
    // TODO: Find out what exactly the EN pin does.  It is mapped to a connector on the motor shield board
    // but the SparkFun example Arduino code does nothing with the pin that is mapped/connected to it.
    pinMode(PIN_EN, INPUT);
    pinMode(PIN_CS, INPUT);

    pinMode(PIN_PROGRAM_SWITCH_1, INPUT_PULLUP);
    pinMode(PIN_PROGRAM_SWITCH_2, INPUT_PULLUP);
    pinMode(PIN_PARK_TRIGGER, INPUT_PULLUP);
    pinMode(PIN_PARK_FEEDBACK, INPUT_PULLUP);

    motor1State = new MotorState(PIN_INA, PIN_INB, PIN_PWM);
    for (int i=0; i<2; i++) {
        for (int j=0; j<2; j++) {
            programs[i][j] = new Program();
        }
    }

    /*
     * "park" signal that is activated by some kind of trigger switch on PIN_PARK_TRIGGER
     * and stays active until the motor reaches the park position and switches PIN_PARK_FEEDBACK
     */
    Signal *signalParkMotorFeedback = new SignalParkMotorFeedback(PIN_PARK_TRIGGER, PIN_PARK_FEEDBACK);

    /*
     * Signal that reports active whenever the PIN_PARK_TRIGGER (e.g. Magnetic Switch)
     * input is connected / "on" / LOW
     */
    Signal *signalParkTrigger = new SignalInputPin(PIN_PARK_TRIGGER);
    /*
     * Signal that is always active if the motor is NOT in the park position
     * The "on" value is reversed so that the signal is active when the actual
     * pin at its default/"off"/unconnected/HIGH value
     */
    Signal *signalMotorNotParked = new SignalInputPin(PIN_PARK_FEEDBACK, HIGH);
    /*
     * The motor needs a tiny bit of extra time to get to the center of the
     * park position after the switch signals that the park position has been
     * reached.  This can be tuned to match reality.
     */
    signalMotorNotParked->setExtraTimeDuration(65);


    // Diagnostic/Debugging.  Why does the program start ok when the first step
    // is NOT the motor-park reset step?
    programs[0][0]->addProgramStep(new ProgramStepWithDuration(BRAKEGND, 0, 100));
    // Reset back to the "park" position to run the other steps starting from the same place
    programs[0][0]->addProgramStep(new ProgramStepWithSignal(FORWARD, 200, signalMotorNotParked));
    programs[0][0]->addProgramStep(new ProgramStepWithDuration(BRAKEGND, 0, 3000));
    // Back and forth a bit slower
    programs[0][0]->addProgramStep(new ProgramStepWithDuration(FORWARD, 70, 1000));
    programs[0][0]->addProgramStep(new ProgramStepWithDuration(REVERSE, 70, 1200));
    programs[0][0]->addProgramStep(new ProgramStepWithDuration(FORWARD, 70, 750));
    programs[0][0]->addProgramStep(new ProgramStepWithDuration(REVERSE, 70, 1200));
    programs[0][0]->addProgramStep(new ProgramStepWithDuration(FORWARD, 70, 750));
    programs[0][0]->addProgramStep(new ProgramStepWithDuration(REVERSE, 70, 1000));
    // off for a few seconds
    programs[0][0]->addProgramStep(new ProgramStepWithDuration(BRAKEGND, 0, 1500));
    // reverse just a bit
    programs[0][0]->addProgramStep(new ProgramStepWithDuration(REVERSE, 70, 500));
    // back and forth several times quick (slightly longer forward to process a bit)
    programs[0][0]->addProgramStep(new ProgramStepWithDuration(REVERSE, 120, 150));
    programs[0][0]->addProgramStep(new ProgramStepWithDuration(FORWARD, 120, 250));
    programs[0][0]->addProgramStep(new ProgramStepWithDuration(REVERSE, 120, 150));
    programs[0][0]->addProgramStep(new ProgramStepWithDuration(FORWARD, 120, 250));
    programs[0][0]->addProgramStep(new ProgramStepWithDuration(REVERSE, 120, 150));
    programs[0][0]->addProgramStep(new ProgramStepWithDuration(FORWARD, 120, 250));
    programs[0][0]->addProgramStep(new ProgramStepWithDuration(REVERSE, 120, 150));
    programs[0][0]->addProgramStep(new ProgramStepWithDuration(FORWARD, 120, 250));
    // off for a second
    programs[0][0]->addProgramStep(new ProgramStepWithDuration(BRAKEGND, 0, 1000));
    // back and forth several times a little slower (slightly longer in reverse to precess a bit)
    programs[0][0]->addProgramStep(new ProgramStepWithDuration(REVERSE, 90, 350));
    programs[0][0]->addProgramStep(new ProgramStepWithDuration(FORWARD, 90, 250));
    programs[0][0]->addProgramStep(new ProgramStepWithDuration(REVERSE, 90, 350));
    programs[0][0]->addProgramStep(new ProgramStepWithDuration(FORWARD, 90, 250));
    programs[0][0]->addProgramStep(new ProgramStepWithDuration(REVERSE, 90, 350));
    programs[0][0]->addProgramStep(new ProgramStepWithDuration(FORWARD, 90, 250));
    programs[0][0]->addProgramStep(new ProgramStepWithDuration(REVERSE, 90, 350));
    programs[0][0]->addProgramStep(new ProgramStepWithDuration(FORWARD, 90, 250));

    // PIN_PROGRAM_SWITCH_1 SWITCH ONLY
    // Wait 5s, reverse 5s, wait 5s, forward 5s, repeat
    programs[1][0]->addProgramStep(new ProgramStepWithDuration(BRAKEGND, 0, 5000));
    programs[1][0]->addProgramStep(new ProgramStepWithDuration(REVERSE, 200, 5000));
    programs[1][0]->addProgramStep(new ProgramStepWithDuration(BRAKEGND, 0, 5000));
    programs[1][0]->addProgramStep(new ProgramStepWithDuration(FORWARD, 200, 5000));

    // PIN_PROGRAM_SWITCH_2 SWITCH ONLY
    // Cycle through different pwm settings, increasing with each step (motor controller PWM handling test)
    programs[0][1]->addProgramStep(new ProgramStepWithDuration(FORWARD, 15, 500));
    programs[0][1]->addProgramStep(new ProgramStepWithDuration(BRAKEGND, 0, 500));
    programs[0][1]->addProgramStep(new ProgramStepWithDuration(FORWARD, 35, 500));
    programs[0][1]->addProgramStep(new ProgramStepWithDuration(BRAKEGND, 0, 500));
    programs[0][1]->addProgramStep(new ProgramStepWithDuration(FORWARD, 70, 500));
    programs[0][1]->addProgramStep(new ProgramStepWithDuration(BRAKEGND, 0, 500));
    programs[0][1]->addProgramStep(new ProgramStepWithDuration(FORWARD, 105, 500));
    programs[0][1]->addProgramStep(new ProgramStepWithDuration(BRAKEGND, 0, 500));
    programs[0][1]->addProgramStep(new ProgramStepWithDuration(FORWARD, 140, 500));
    programs[0][1]->addProgramStep(new ProgramStepWithDuration(BRAKEGND, 0, 500));
    programs[0][1]->addProgramStep(new ProgramStepWithDuration(FORWARD, 175, 500));
    programs[0][1]->addProgramStep(new ProgramStepWithDuration(BRAKEGND, 0, 500));
    programs[0][1]->addProgramStep(new ProgramStepWithDuration(FORWARD, 210, 500));
    programs[0][1]->addProgramStep(new ProgramStepWithDuration(BRAKEGND, 0, 500));
    programs[0][1]->addProgramStep(new ProgramStepWithDuration(FORWARD, 245, 500));
    programs[0][1]->addProgramStep(new ProgramStepWithDuration(BRAKEGND, 0, 500));
    programs[0][1]->addProgramStep(new ProgramStepWithDuration(FORWARD, 255, 500));
    programs[0][1]->addProgramStep(new ProgramStepWithDuration(BRAKEGND, 0, 3000));

    // BOTH PIN_PROGRAM_SWITCH_*'s
    // Move the motor forward until the motor's park switch signals it is in the park position
    programs[1][1]->addProgramStep(new ProgramStepWithSignal(FORWARD, 200, signalParkMotorFeedback));
    // Stop the motor and keep it that way until the park trigger is released.
    programs[1][1]->addProgramStep(new ProgramStepWithSignal(BRAKEGND, 0, signalParkTrigger));
    // Previous steps with Signal get skipped if the signal is not on.
    // The remaining steps get skipped if the signal _IS_ on
    programs[1][1]->addProgramStep(new ProgramStepWithDuration(BRAKEGND, 0, 5000, signalParkTrigger));
    // Long running step to prove that the trigger makes it cut short.
    programs[1][1]->addProgramStep(new ProgramStepWithDuration(FORWARD, 105, 10000, signalParkTrigger));

    // Initialize motor to stopped state
    motor1State->stopMotor();

    // TODO: Find out why enabling Serial comms causes the onboard LED to just blink.
    Serial.begin(9600);
    Serial.println(F("Automated Jigging Machine is ready to catch fish."));
    for (int i=0; i<2; i++) {
    	for (int j=0; j<2; j++) {
    	    Serial.print(F("Program ["));
    	    Serial.print(i);
    	    Serial.print(F("]["));
    	    Serial.print(j);
    	    Serial.print(F("] has "));
    	    Serial.print(programs[i][j]->getProgramStepCount());
    	    Serial.println(F(" steps."));
    	}
    }
}

void loop() {
    if (motor1State->isOvercurrentFaultDetected()) {
        // fast-blink the on-board LED
        digitalWrite(PIN_STATUS, HIGH);
        delay(50);
        digitalWrite(PIN_STATUS, LOW);
        delay(50);
    } else {
        // Check every loop to be sure the motor isn't stalled
        // or drawing too much current.
    	if (motor1State->checkOvercurrent(CS_THRESHOLD)) {
            Serial.println(F("Motor fault current level detected.  Shutting down now."));
            Serial.print(F("Last current sense pin value before shutdown was "));
            Serial.println(motor1State->getLastCurrentSensePinValue());
        } else {
            // continue the motor-sequence program
            digitalWrite(PIN_STATUS, HIGH);
            //delay(UPDATE_INTERVAL);
            // Note: If these are thought of as "bits", the boolean algebra is reversed
            // because the pins are initialized with the pullup enabled so "on" == LOW
            uint8_t programSwitch1 = (digitalRead(PIN_PROGRAM_SWITCH_1) == HIGH) ? 0 : 1;
            uint8_t programSwitch2 = (digitalRead(PIN_PROGRAM_SWITCH_2) == HIGH) ? 0 : 1;
            programs[programSwitch1][programSwitch2]->updateProgramState(motor1State);
            motor1State->updateMotorState();
        }
    }

}
