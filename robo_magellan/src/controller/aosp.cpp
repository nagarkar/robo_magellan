/*
 * aosp.cpp
 *
 *  Created on: Jun 24, 2018
 *      Author: chinm_000
 */

#include "aosp.h"
#include <iostream>

using namespace std;

static bool callback_registered = false;
static bool callback_active 	= false;

static void _AOSP_DSM_Callback(void);

/*
void AOSP_DSM_Connected() {
	if (BSP_DSM_IsUp()) {
	    double throttle_value = BSP_DSM_Get(1);
	    double direction_value = BSP_DSM_Get(2);
	    double motor_speeds[2];
	    GetDiffDriveMotorSpeeds(throttle_value, direction_value, motor_speeds);
	}
}
*/

void AOSP_DSM_Callback_Deactivate() {
	callback_active = false;
}

void AOSP_DSM_Callback_Activate() {
	if (!callback_registered) {
		BSP_Warn_on_failure(BSP_MOTORS_Setup(), "FATAL: Could not initialize motors");
		BSP_Warn_on_failure(BSP_DSM_Callback(_AOSP_DSM_Callback), "FATAL: Could not setup DSM callback");
	}
	callback_active = true;
}

void _AOSP_DSM_Callback(void) {
	double throttle = BSP_DSM_Get(AOSP_THROTTLE_CHANNEL);
	double direction = BSP_DSM_Get(AOSP_DIRECTION_CHANNEL);
	bool auto_mode = BSP_DSM_Get(AOSP_AUTO_MODE_CHANNEL) == AUTO_MODE_ON ? true : false;
	double speed = BSP_DSM_Get(AOSP_SPEED_CHANNEL);
    double motor_speeds[2] = {0.0, 0.0};
    if (!auto_mode) {
    	AOSP_ComputeDiffDriveMotorSpeeds(throttle, direction, speed, motor_speeds);
    	AOSP_RunDiffDriveMotors(motor_speeds);
    }
}

void AOSP_RunDiffDriveMotors(double * motor_speeds) {
	BSP_MOTORS_Set(LEFT_MOTORS, 	DIFF_DRIVE_LEFT_MOTOR_POLARITY * motor_speeds[DIFF_DRIVE_LEFT_MOTOR_PORT]);
	BSP_MOTORS_Set(RIGHT_MOTORS, 	DIFF_DRIVE_RGHT_MOTOR_POLARITY * motor_speeds[DIFF_DRIVE_RGHT_MOTOR_PORT]);
}

/**
 * @param motor_speeds Left and Right motor speeds in interval (-1, 1).
 */

void AOSP_ComputeDiffDriveMotorSpeeds(double throttle, double direction, double _speed, double* motor_speeds) {

	double scaled_speed = ABS_MAX_SPEED * _speed;

	double left_multiplier, right_multiplier = 0;

	// Forward and Reverse are mutually exclusive, but it is possible neither is set.
	bool forward 	= IS_FWD(throttle);
	bool reverse 	= IS_REV(throttle);
	cout << "Got throttle:" << (forward ? "FORWARD" : (reverse ? "REVERSE" : "NONE")) << "(" << throttle << ")" <<  endl;

	// Right and Left are mutually exclusive
	bool go_right 	= IS_RGHT(direction);
	bool go_left 	= IS_LEFT(direction);

	cout << "Got direction:" << (go_right ? "RIGHT" : (go_left ? "LEFT": "NONE")) << "(" << direction << ")" << endl;


	if (forward || reverse) {
		int multiplier = forward ? 1 : -1;
		if (!go_left && !go_right) {
			left_multiplier = multiplier;
			right_multiplier = multiplier;
		} else if (go_left) {
			right_multiplier = multiplier;
			//left_speed = -0.5 * multiplier;
			left_multiplier = -multiplier;
		} else if (go_right) {
			//right_multiplier = -0.5 * multiplier;
			right_multiplier = -multiplier;
			left_multiplier = multiplier;
		}
	} else {
		left_multiplier = 0;
		left_multiplier = 0;
	}

	double left_speed = left_multiplier * scaled_speed;
	double right_speed = right_multiplier * scaled_speed;
	//TODO: Assert left_speed and right_speed are between {-1,1}
	motor_speeds[DIFF_DRIVE_LEFT_MOTOR_PORT] = CONSTRAIN(left_speed,  -ABS_MAX_SPEED, ABS_MAX_SPEED);
	motor_speeds[DIFF_DRIVE_RGHT_MOTOR_PORT] = CONSTRAIN(right_speed, -ABS_MAX_SPEED, ABS_MAX_SPEED);

}
