/* 
 This code and the corresponding pin choices are intended for Arduino NANO v3.0
 
 Motor controller is a TE466 "mini" Monster Moto Shield.  
 Note: The TE466 is a MUCH cheaper (~$8) single-motor version of the dual-motor SparkFun Monster Moto Shield (~$70)
 
 Note: INA and INB control the direction (or braking) according to whether they're set HIGH or LOW.
    See details in the comments for the startMotor() function below.

 Note: This was adapted (a little bit) from the example code for the SparkFun
    (2-chip, 2-motor) Monster Moto Shield.
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
    uint8_t motorSpeed = 0;
    long durationMillis = 0;
    long startTime = 0;
    
    // derived upon construction
    uint8_t desiredStateIna;
    uint8_t desiredStateInb;
    
  public:
    ProgramStep(uint8_t motorDirection, uint8_t motorSpeed, long durationMillis) {
        this->motorDirection = motorDirection;
        this->motorSpeed = motorSpeed;
        this->durationMillis = durationMillis;
        if (motorDirection == FORWARD || motorDirection == BRAKEVCC)
            desiredStateIna = HIGH;
        else
            desiredStateIna= LOW;

        if ((motorDirection == BRAKEVCC) || (motorDirection == REVERSE))
            desiredStateInb = HIGH;
        else
            desiredStateInb = LOW;
    }

    boolean activate(MotorState *motorState) {
        startTime = millis();  // reset the clock
        motorState->setDesiredState(desiredStateIna, desiredStateInb, motorSpeed);
    }
    
    boolean isDurationComplete() {
        long endTime = startTime + durationMillis;
        return (millis() >= endTime);
    }
    
};

class Program {
    uint8_t currentProgramStepIndex = 0;
    uint8_t programStepCount = 0;
    boolean programStarted = false;
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

    void updateProgramState(MotorState *motorState) {
        boolean stepChanged = false;
        // update the index for the current program step when the current step is done    
        if (programSteps[currentProgramStepIndex]->isDurationComplete()) {
            stepChanged = true;
            currentProgramStepIndex++;
            if (currentProgramStepIndex >= programStepCount) {
                currentProgramStepIndex = 0;  // loop and repeat program
            }
        }
        if (stepChanged || !programStarted) {
            programStarted = true;
            programSteps[currentProgramStepIndex]->activate(motorState);
        }
    }
};

MotorState *motor1State;

Program *currentProgram;

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

    motor1State = new MotorState();
    currentProgram = new Program();
    
    // on full back and forth (4 seconds total)
    currentProgram->addProgramStep(new ProgramStep(FORWARD, 1023, 2000));
    currentProgram->addProgramStep(new ProgramStep(REVERSE, 1023, 2000));
    // on lower power back and forth (6 seconds total)
    currentProgram->addProgramStep(new ProgramStep(FORWARD, 550, 3000));
    currentProgram->addProgramStep(new ProgramStep(REVERSE, 550, 3000));
    // off for a few seconds
    currentProgram->addProgramStep(new ProgramStep(BRAKEGND, 0, 2000));
    // back and forth 3 times quick
    currentProgram->addProgramStep(new ProgramStep(REVERSE, 550, 250));
    currentProgram->addProgramStep(new ProgramStep(FORWARD, 550, 250));
    currentProgram->addProgramStep(new ProgramStep(REVERSE, 550, 250));
    currentProgram->addProgramStep(new ProgramStep(FORWARD, 550, 250));
    currentProgram->addProgramStep(new ProgramStep(REVERSE, 550, 250));
    currentProgram->addProgramStep(new ProgramStep(FORWARD, 550, 250));
    currentProgram->addProgramStep(new ProgramStep(REVERSE, 550, 250));
    
    // Initialize stopped/braked
    stopMotor();
    
}

void loop() {
    if (motorCurrentFaultDetected) {
        // fast-blink the on-board LED
        digitalWrite(PIN_STATUS, HIGH);
        delay(50);
        digitalWrite(PIN_STATUS, HIGH);
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
            currentProgram->updateProgramState(motor1State);
            motor1State->updateMotorState();
        }
    }
    
}

void stopMotor() {
    digitalWrite(PIN_INA, LOW);
    digitalWrite(PIN_INB, LOW);
    analogWrite(PIN_PWM, 0);
}
