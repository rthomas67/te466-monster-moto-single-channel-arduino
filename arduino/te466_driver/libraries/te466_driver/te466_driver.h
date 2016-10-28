#include "Arduino.h"

/*
 This library implements classes to abstract Arduino interactions with
 a TE466 "mini" Monster Moto Shield.

 Note: The TE466 is a single-motor knock-off the dual-motor SparkFun
 Monster Moto Shield (~$70), but it is usually sold MUCH cheaper (~$8).

 Note: INA and INB control the direction (or braking) according to whether they're set HIGH or LOW.
    See details in the comments for the startMotor() function below.

 Note: This was adapted (a little bit) from the example code for the SparkFun
    (2-chip, 2-motor) Monster Moto Shield.  However, that code is barely workable and
    has some glaring errors:
    For example, that code calls analogWrite on the PWM pin with a value of 1023
    in spite of its own comment that says, correctly, that the value should be
    from 0 to 255.  BTW, using values > 255 with analogWrite() seems to just
    mask the argument value down to whatever its last 8 bits happen to be, so,
    for instance 850 would mask to 11[01010010] or decimal 82, 750 would
    mask to 10[11101110] or decimal 238, and 1023 sould mask to 11[11111111] or
    decimal 255.  So, the actual behavior is the same for 1023 as 255, but other
    values > 255 behave in a way that seems REALLY random.
*/
#ifndef TE466_DRIVER
#define TE466_DRIVER

// Brake to VCC (both motor pins set to HIGH / "on")
#define BRAKEVCC 1
// Run motor forward (pin inA set to HIGH / "on")
#define FORWARD 2
// Run motor backwards (pin INB set to HIGH / "on")
#define REVERSE 3
// Brake to ground (both motor pins set to LOW / "off")
#define BRAKEGND 4

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

// Given the limited SRAM memory (2K on a Mega328),
// program sizes should be kept small.  This defines a
// sensible limit but could be increased for other
// micro-controller chips.
#define MAX_PROGRAM_STEPS 50

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
    int getLastCurrentSensePinValue();

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
    ProgramStep(uint8_t motorDirection, uint8_t motorSpeed);
    virtual ~ProgramStep() {
    	// TODO: Implement default destructor
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

class ProgramStepWithDuration: public ProgramStep {
    unsigned long durationMillis = 0;
    unsigned long startTime = 0;
    Signal *skipSignal = 0;
  public:
    ProgramStepWithDuration(uint8_t motorDirection, uint8_t motorSpeed, long durationMillis);

    ProgramStepWithDuration(uint8_t motorDirection, uint8_t motorSpeed, long durationMillis, Signal *skipSignal);

    boolean isDone();

    boolean activate(MotorState *motorState);

};

class SignalSimulatedWithTimer : public Signal {
    unsigned long startTime;
  public:
    boolean isSignalActive();
    void startTimer();
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
    SignalInputPin(uint8_t digitalPinNumber);
    SignalInputPin(uint8_t digitalPinNumber, uint8_t signalOnValue);
    /*
     * Implemented to report true when the current digital state of the pin
     * matches the value of signalOnValue.
     */
    boolean isSignalActive();

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
    SignalParkMotorFeedback(uint8_t triggerDigitalPinNumber, uint8_t parkStatusDigitalPinNumber);
    SignalParkMotorFeedback(uint8_t triggerDigitalPinNumber, uint8_t parkStatusDigitalPinNumber, uint8_t triggerDigitalPinOn, uint8_t parkStatusDigitalPinOn);
    /*
     * This Signal reports that it is "active" as long as the triggering pin
     * is "on" and the park position pin is still "off" (i.e. not parked yet.)
     */
    boolean isSignalActive();

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
    ProgramStepWithSignal(uint8_t motorDirection, uint8_t motorSpeed, Signal *signalSource);

    /*
     * Returns the false if isSignalActive() returns true.
     * In other words, the program step is _not_ done yet
     * as long as the signal _is_ active.
     */
    boolean isDone();
    boolean activate(MotorState *motorState);
};

class Program {
    uint8_t currentProgramStepIndex = 0;
    uint8_t programStepCount = 0;
    ProgramStep** programSteps = 0;

  public:
    Program();

    void addProgramStep(ProgramStep *programStep);

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
    void updateProgramState(MotorState *motorState);

    uint8_t getProgramStepCount();
};

#endif
