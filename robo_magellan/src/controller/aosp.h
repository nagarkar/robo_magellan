/*
 * aosp.h
 *
 *  Created on: Jun 24, 2018
 *      Author: chinm_000
 */

#pragma once

#include "bsp.h"

/**
 * The BBB has four motor ports. Set the values for ports used for differential drive mode.
 */
#define DIFF_DRIVE_LEFT_MOTOR_PORT	4
#define DIFF_DRIVE_RIGHT_MOTOR_PORT	1

/**
 * Set the value of the absolute valeu of maximum speed. This should be <= 1.
 */
#define ABS_MAX_SPEED 0.8

#define THROTTLE_DEAD_RANGE_MAX  0.5
#define THROTTLE_DEAD_RANGE_MIN -0.5


void AOSP_DSM_Connected();

void GetDiffDriveMotorSpeeds(double throttle, double direction, double* motor_speeds);

