/* 
 This code and the corresponding pin choices are intended for Arduino NANO v3.0
 
 Motor controller is a TE466 "mini" Monster Moto Shield.  
 Note: The TE466 is a MUCH cheaper (~$8) single-motor version of the dual-motor SparkFun Monster Moto Shield (~$70)
 
 Note: INA and INB control the direction (or braking) according to whether they're set HIGH or LOW.
    See details in the comments for the startMotor() function below.

 Note: This was adapted (a little bit) from the example code for the SparkFun
    (2-chip, 2-motor) Monster Moto Shield.  However, that code is barely workable and
    has glaring errors (e.g. calling analogWrite with a value > 255 in spite of
    the comment that says otherwise).
 Date: 2016/10/10
 Author: R Thomas
 
 Reminder: digitalWrite(someAnalogPin, HIGH) turns on the built-in pull-up resistor on an analog pin
 Reminder: analogWrite(someDigitalPin, value) enables PWM output on a digital pin (if capable)
 */

// Brake to VCC (both motor pins set to HIGH / "on")
#define BRAKEVCC 1
// Run motor forward (pin inA set to HIGH / "on")
#define FORWARD 2
// Run motor backwards (pin INB set to HIGH / "on")
#define REVERSE 3
// Brake to ground (both motor pins set to LOW / "off")
#define BRAKEGND 4
#define CS_THRESHOLD 600
#define UPDATE_INTERVAL 10

//#define PWM_SPEED_5 13
//#define PWM_SPEED_10 26
//#define PWM_SPEED_15 38
//#define PWM_SPEED_20 51
//#define PWM_SPEED_25 64
//#define PWM_SPEED_30 77
//#define PWM_SPEED_35 89
//#define PWM_SPEED_40 102
//#define PWM_SPEED_45 115
//#define PWM_SPEED_50 127
//#define PWM_SPEED_55 140
//#define PWM_SPEED_60 153
//#define PWM_SPEED_65 166
//#define PWM_SPEED_70 179
//#define PWM_SPEED_75 191
//#define PWM_SPEED_80 204
//#define PWM_SPEED_85 217
//#define PWM_SPEED_90 230
//#define PWM_SPEED_95 242
//#define PWM_SPEED_100 255

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

static const uint8_t MAXIMUM_PROGRAM_STEPS = 100;

boolean motorCurrentFaultDetected = false;

class MotorState {
    uint8_t currentStateIna = LOW;
    uint8_t currentStateInb = LOW;
    uint8_t currentStatePwm = 0;
    uint8_t desiredStateIna = LOW;
    uint8_t desiredStateInb = LOW;
    uint8_t desiredStatePwm = 0;
    
  public:
    MotorState() {
      
    }
    
    void setDesiredState(uint8_t ina, uint8_t inb, uint8_t pwm) {
        desiredStateIna = ina;
        desiredStateInb = inb;
        desiredStatePwm = pwm;
    }
    
    void updateMotorState() {
        if (desiredStateIna != currentStateIna) {
            currentStateIna = desiredStateIna;
            digitalWrite(PIN_INA, currentStateIna);
        }
        if (desiredStateInb != currentStateInb) {
            currentStateInb = desiredStateInb;
            digitalWrite(PIN_INB, currentStateInb);
        }
        if (desiredStatePwm != currentStatePwm) {
            currentStatePwm = desiredStatePwm;
            analogWrite(PIN_PWM, currentStatePwm);
        }
  }

};

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
    ProgramStep(uint8_t motorDirection, uint8_t motorSpeed) {
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
    
    virtual boolean activate(MotorState *motorState) = 0;
    
    virtual boolean isDone(void) = 0;
    
};

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
    long extraTimeDuration = 0;
    long extraTimeStartTime = 0;
    protected:
    void startExtraTime() {
        extraTimeStartTime = millis();
    }
    boolean isDuringExtraTime() {
        return (millis() <= (extraTimeStartTime + extraTimeDuration));
    }

  public:  
    void setExtraTimeDuration(uint8_t extraTimeDuration) {
        this->extraTimeDuration = extraTimeDuration;
    }
    virtual boolean isSignalActive() = 0;
    
};

class ProgramStepWithDuration: public ProgramStep {
    long durationMillis = 0;
    long startTime = 0;
    Signal *skipSignal = 0;
  public:
    ProgramStepWithDuration(uint8_t motorDirection, uint8_t motorSpeed, long durationMillis) : ProgramStep(motorDirection, motorSpeed) {
        this->durationMillis = durationMillis;
    }
    
    ProgramStepWithDuration(uint8_t motorDirection, uint8_t motorSpeed, long durationMillis, Signal *skipSignal) : ProgramStepWithDuration(motorDirection, motorSpeed, durationMillis) {
        this->skipSignal = skipSignal;
    }
    
    boolean isDone() {
        /*
         * If this object was constructed with a Signal object, any time that signal
         * reports that it is "on", this step immediately reports that it is done.
         */
        if (skipSignal && skipSignal->isSignalActive()) {
            return true;
        }
        long endTime = startTime + durationMillis;
        return (millis() >= endTime);
    }

    boolean activate(MotorState *motorState) {
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
    
};

class SignalSimulatedWithTimer : public Signal {
    long startTime;
  public:
    boolean isSignalActive() {
        return (millis() > (startTime + 3000));
    }
    
    void startTimer() {
        startTime = millis();
    }
};

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
    SignalInputPin(uint8_t digitalPinNumber) : Signal() {
        this->digitalPinNumber = digitalPinNumber;
    }
    SignalInputPin(uint8_t digitalPinNumber, uint8_t signalOnValue) : SignalInputPin(digitalPinNumber) {
        this->signalOnValue = signalOnValue;
    }
    /*
     * Implemented to report true when the current digital state of the pin
     * matches the value of signalOnValue.
     */
    boolean isSignalActive() {
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
    
};


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
    SignalParkMotorFeedback(uint8_t triggerDigitalPinNumber, uint8_t parkStatusDigitalPinNumber) : Signal() {
        this->triggerDigitalPinNumber = triggerDigitalPinNumber;
        this->parkStatusDigitalPinNumber = parkStatusDigitalPinNumber;
    }
    SignalParkMotorFeedback(uint8_t triggerDigitalPinNumber, uint8_t parkStatusDigitalPinNumber, uint8_t triggerDigitalPinOn, uint8_t parkStatusDigitalPinOn)
            : SignalParkMotorFeedback(triggerDigitalPinNumber, parkStatusDigitalPinNumber) {
        this->triggerDigitalPinOn = triggerDigitalPinOn;
        this->parkStatusDigitalPinOn = parkStatusDigitalPinOn;
    }
    /*
     * This Signal reports that it is "active" as long as the triggering pin
     * is "on" and the park position pin is still "off" (i.e. not parked yet.)
     */
    boolean isSignalActive() {
        return (digitalRead(triggerDigitalPinNumber) == triggerDigitalPinOn 
            && digitalRead(parkStatusDigitalPinNumber) != parkStatusDigitalPinOn);
    }
    
};

/**
 * Note: This can be used as a "pause until signal is not active"
 * by simply never calling activate, or it may follow another
 * signal step with the same signal but different motor settings.
 */
class ProgramStepWithSignal: public ProgramStep {
  protected:
    Signal *signalSource = 0;

  public:
    ProgramStepWithSignal(uint8_t motorDirection, uint8_t motorSpeed, Signal *signalSource) : ProgramStep(motorDirection, motorSpeed) {
        this->signalSource = signalSource;
    }
    
    /*
     * Returns the false if isSignalActive() returns true.
     * In other words, the program step is _not_ done yet
     * as long as the signal _is_ active.
     */
    boolean isDone() {
        return (signalSource->isSignalActive() != true);
    }
    
    boolean activate(MotorState *motorState) {
        if (signalSource && signalSource->isSignalActive()) {
            motorState->setDesiredState(desiredStateIna, desiredStateInb, motorSpeed);
        }
        // Otherwise don't do anything
    }
};

class Program {
    uint8_t currentProgramStepIndex = 0;
    uint8_t programStepCount = 0;
    ProgramStep *programSteps[MAXIMUM_PROGRAM_STEPS];
        
  public:
    Program() {
    }
        
    void addProgramStep(ProgramStep *programStep) {
        // Check current programStepCount (1-based) against maximum
        if (programStepCount < MAXIMUM_PROGRAM_STEPS)  {
            programStepCount++;
            // array index is 0-based
            programSteps[programStepCount - 1] = programStep;
        }
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
    void updateProgramState(MotorState *motorState) {
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
};

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

    motor1State = new MotorState();
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
    
    
    // Initialize stopped/braked
    stopMotor();
    
}

void loop() {
    if (motorCurrentFaultDetected) {
        // fast-blink the on-board LED
        digitalWrite(PIN_STATUS, HIGH);
        delay(50);
        digitalWrite(PIN_STATUS, LOW);
        delay(50);
    } else {
        // Check every loop to be sure the motor isn't stalled
        // or drawing too much current.
        if (analogRead(PIN_CS) > CS_THRESHOLD) {
            stopMotor();
            motorCurrentFaultDetected = true;
        } else {
            // continue the motor-sequence program
            digitalWrite(PIN_STATUS, HIGH);
            delay(UPDATE_INTERVAL);
            // Note: If these are thought of as "bits", the boolean algebra is reversed
            // because the pins are initialized with the pullup enabled so "on" == LOW
            uint8_t programSwitch1 = (digitalRead(PIN_PROGRAM_SWITCH_1) == HIGH) ? 0 : 1;
            uint8_t programSwitch2 = (digitalRead(PIN_PROGRAM_SWITCH_2) == HIGH) ? 0 : 1;
            programs[programSwitch1][programSwitch2]->updateProgramState(motor1State);
            motor1State->updateMotorState();
        }
    }
    
}

void stopMotor() {
    digitalWrite(PIN_INA, LOW);
    digitalWrite(PIN_INB, LOW);
    analogWrite(PIN_PWM, 0);
}
