#include <Arduino.h>
#include "Te466Common.h"
#include "MotorSequenceProgram.h"
#include "MotorState.h"
#include "Signal.h"
#include "SignalInputPin.h"
#include "SignalParkMotorFeedback.h"
#include "SignalSimulatedWithTimer.h"
#include "ProgramStep.h"
#include "ProgramStepWithDuration.h"
#include "ProgramStepWithSignal.h"

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


#endif
