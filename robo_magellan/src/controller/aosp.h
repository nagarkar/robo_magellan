/*
 * aosp.h
 *
 *  Created on: Jun 24, 2018
 *      Author: chinm_000
 */

#pragma once
#include "bsp.h"

/**** Motor Settings ***/

// The BBB has four motor ports. Set the values for ports used for differential drive mode.
#define DIFF_DRIVE_LEFT_MOTOR_PORT	0
#define DIFF_DRIVE_RGHT_MOTOR_PORT	1

#define DIFF_DRIVE_LEFT_MOTOR_POLARITY	 1
#define DIFF_DRIVE_RGHT_MOTOR_POLARITY	 1

/**
 * Set the value of the absolute value of maximum speed. This should be <= 1.
 * Pick this based on motor amperage draw, your driver's amperage rating, and
 **/
#define ROBOT_MOTOR_MAX_VOLTS 7.5
#define MOTOR_BATT_MAX_VOLTS  13
#define ABS_MAX_SPEED (ROBOT_MOTOR_MAX_VOLTS/MOTOR_BATT_MAX_VOLTS)

#define SIGN(x) (signbit(x) ? -1 : 1)

#define CONSTRAIN(x, min, max) (x > max ? max : (x < min ? min : x))

/***** DSM Settings *****/
#define AOSP_SPEED_CHANNEL 	((int)1)
#define AOSP_THROTTLE_CHANNEL 	((int)3)
#define AOSP_DIRECTION_CHANNEL 	((int)2)
#define AOSP_AUTO_MODE_CHANNEL 	((int)5)

#define THROTTLE_DEAD_RANGE_MAX  0.5
#define THROTTLE_DEAD_RANGE_MIN  -0.5

#define DIRECTION_DEAD_RANGE_MAX  0.5
#define DIRECTION_DEAD_RANGE_MIN -0.5

#define IS_RGHT(direction)  { direction < DIRECTION_DEAD_RANGE_MIN }
#define IS_LEFT(direction)  { direction > DIRECTION_DEAD_RANGE_MAX }

#define IS_FWD(throttle) { throttle > THROTTLE_DEAD_RANGE_MAX }
#define IS_REV(throttle) { throttle < THROTTLE_DEAD_RANGE_MIN }

#define AUTO_MODE_OFF 		((int)0)
#define AUTO_MODE_ON 		((int)1)


void AOSP_DSM_Callback_Deactivate();
void AOSP_DSM_Callback_Activate();

void AOSP_ComputeDiffDriveMotorSpeeds(double throttle, double direction, double speed, double* motor_speeds);

void AOSP_RunDiffDriveMotors(double * motor_speeds);
