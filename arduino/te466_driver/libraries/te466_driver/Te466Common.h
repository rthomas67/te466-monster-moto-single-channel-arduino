/*
 * Te466Common.h
 *
 *  Created on: Nov 25, 2016
 *      Author: rrt
 */

#ifndef LIBRARIES_TE466_DRIVER_TE466COMMON_H_
#define LIBRARIES_TE466_DRIVER_TE466COMMON_H_

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

#endif /* LIBRARIES_TE466_DRIVER_TE466COMMON_H_ */
