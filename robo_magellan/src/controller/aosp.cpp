/*
 * aosp.cpp
 *
 *  Created on: Jun 24, 2018
 *      Author: chinm_000
 */

#include "aosp.h"

void AOSP_DSM_Connected() {
	if (BSP_DSM_IsUp()) {
	    double throttle_value = BSP_DSM_Get(1);
	    double direction_value = BSP_DSM_Get(2);
	    double motor_speeds[2];
	    GetDiffDriveMotorSpeeds(throttle_value, direction_value, motor_speeds);
	}
}

/**
 * @param motor_speeds Left and Right motor speeds in interval (-1, 1).
 */
void GetDiffDriveMotorSpeeds(double throttle, double direction, double* motor_speeds) {

	double left_speed, right_speed = 0;

	// Forward and Reverse are mutually exclusive
	bool forward 	= throttle 	> THROTTLE_DEAD_RANGE_MAX;
	bool reverse 	= throttle 	< THROTTLE_DEAD_RANGE_MIN;

	// Right and Left are mutually exclusive
	bool go_right 	= direction > THROTTLE_DEAD_RANGE_MAX;
	bool go_left 	= direction < THROTTLE_DEAD_RANGE_MIN;

	if (forward || reverse) {
		int multiplier = forward ? 1 : -1;
		if (!go_left && !go_right) {
			left_speed = multiplier;
			right_speed = multiplier;
		} else if (go_left) {
			right_speed = multiplier;
		} else if (go_right) {
			left_speed = multiplier;
		}
	} else {
		left_speed = 0;
		left_speed = 0;
	}

	left_speed 	= left_speed * ABS_MAX_SPEED;
	right_speed = right_speed * ABS_MAX_SPEED;

	motor_speeds[DIFF_DRIVE_LEFT_MOTOR_PORT] = left_speed;
	motor_speeds[DIFF_DRIVE_RIGHT_MOTOR_PORT] = right_speed;

}
