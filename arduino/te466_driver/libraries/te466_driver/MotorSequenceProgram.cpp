#include "MotorSequenceProgram.h"
/*
 * MotorSequenceProgram.cpp
 *
 *  Created on: Nov 25, 2016
 *      Author: rrt
 */

MotorSequenceProgram::MotorSequenceProgram() {
}

MotorSequenceProgram::~MotorSequenceProgram() {
    delete [] programSteps;
}

/**
 * Note: This takes on responsibility for deleting the memory for
 * programStep since it is passed by reference.  This also presumes
 * that programStep was allocated using new so that it is a heap
 * allocated object.
 */
void MotorSequenceProgram::addProgramStep(ProgramStep *programStep) {
    // Create or expand the array of program steps
    ProgramStep** programStepsOld = programSteps;
    programSteps = new ProgramStep*[programStepCount + 1];
    // copy pointers from the old array to the expanded array
    if (programStepsOld != 0) {
        for (int i=0; i<programStepCount; i++) {
            programSteps[i] = programStepsOld[i];
            programStepsOld[i] = 0;  // prevent destructor call on delete[]
        }
    }
    delete [] programStepsOld;
    // array index is 0-based but the array is one larger now, so
    programSteps[programStepCount] = programStep;
    // increase this for next time
    programStepCount++;
}

uint8_t MotorSequenceProgram::getProgramStepCount() {
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
void MotorSequenceProgram::updateProgramState(MotorState *motorState) {
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
