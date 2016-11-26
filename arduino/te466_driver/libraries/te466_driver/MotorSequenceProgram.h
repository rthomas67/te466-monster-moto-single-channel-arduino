#include "ProgramStep.h"
#include "MotorState.h"

#ifndef LIBRARIES_TE466_DRIVER_MOTORSEQUENCEPROGRAM_H_
#define LIBRARIES_TE466_DRIVER_MOTORSEQUENCEPROGRAM_H_


class MotorSequenceProgram {
    uint8_t currentProgramStepIndex = 0;
    uint8_t programStepCount = 0;
    ProgramStep** programSteps = 0;

  public:
    MotorSequenceProgram();
    ~MotorSequenceProgram();

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
