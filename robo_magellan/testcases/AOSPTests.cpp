#include "gtest/gtest.h"
#include "controller/bsp.h"
#include "roboticscape.h"
#include "controller/aosp.h"

#define INV_SQRT_ABS_ERR .01f

#define EXPECTED_CHANNELS 6
#define EXPECTED_RESOLUTION 1024

// For turning, ratio of speeds between dominant side and the other side
#define TURNING_DIFF_RATIO 1

using namespace testing;
using namespace std;

#define FWD 0
#define REV 1
#define DEFAULT_DIR 0 // same as FWD

#define DEFAULT_SPEED 1

#define ASSERT_MOTOR_DIRECTION_PINS(left, right) { \
	ASSERT_FLOAT_EQ(rc_gpio_get_value(MOTOR_DIR_PIN_CHIP, MOTOR_DIR_LEFT_PIN), LEFT_MOTOR_POLARITY_REVERSED ^ left); \
	ASSERT_FLOAT_EQ(rc_gpio_get_value(MOTOR_DIR_PIN_CHIP, MOTOR_DIR_RGHT_PIN), RIGHT_MOTOR_POLARITY_REVERSED ^ right); \
}

namespace {
	class AOSPTests : public Test {
	protected:
		static bool bsp_setup_done;
		void SetUp() {
			if (!bsp_setup_done) {
				ASSERT_TRUE(BSP_Setup(BSP_DONT_PRINT_OPTIONS));
				ASSERT_TRUE(BSP_PWM_Setup());
				ASSERT_TRUE(BSP_MOTORS_Setup());
				bsp_setup_done = true;
			}
		}
		void TearDown() {}
	};

	double speeds[2] = {0, 0};
	TEST_F(AOSPTests, dead_range_max_throttle) {
		AOSP_ComputeDiffDriveMotorSpeeds(THROTTLE_DEAD_RANGE_MAX, DIRECTION_DEAD_RANGE_MAX, DEFAULT_SPEED, speeds);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_LEFT_MOTOR_PORT], 0);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_RGHT_MOTOR_PORT], 0);

		AOSP_RunDiffDriveMotors(speeds);
		ASSERT_MOTOR_DIRECTION_PINS(DEFAULT_DIR, DEFAULT_DIR);
	}

	TEST_F(AOSPTests, dead_range_min_throttle) {
		AOSP_ComputeDiffDriveMotorSpeeds(THROTTLE_DEAD_RANGE_MIN, DIRECTION_DEAD_RANGE_MIN, DEFAULT_SPEED, speeds);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_LEFT_MOTOR_PORT], 0);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_RGHT_MOTOR_PORT], 0);

		AOSP_RunDiffDriveMotors(speeds);
		ASSERT_MOTOR_DIRECTION_PINS(DEFAULT_DIR, DEFAULT_DIR);
	}

	TEST_F(AOSPTests, forward) {
		AOSP_ComputeDiffDriveMotorSpeeds(THROTTLE_DEAD_RANGE_MAX + 0.1, DIRECTION_DEAD_RANGE_MIN, DEFAULT_SPEED, speeds);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_LEFT_MOTOR_PORT],  ABS_MAX_SPEED);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_RGHT_MOTOR_PORT], ABS_MAX_SPEED);

		AOSP_RunDiffDriveMotors(speeds);
		ASSERT_MOTOR_DIRECTION_PINS(FWD, FWD);
	}

	TEST_F(AOSPTests, reverse) {
		AOSP_ComputeDiffDriveMotorSpeeds(THROTTLE_DEAD_RANGE_MIN - 0.1, DIRECTION_DEAD_RANGE_MIN, DEFAULT_SPEED, speeds);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_LEFT_MOTOR_PORT],  -ABS_MAX_SPEED);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_RGHT_MOTOR_PORT], -ABS_MAX_SPEED);

		AOSP_RunDiffDriveMotors(speeds);
		ASSERT_MOTOR_DIRECTION_PINS(REV, REV);
	}

#define RGHT_DIRECTION_VALUE 	(DIRECTION_DEAD_RANGE_MIN - 0.1)
#define LEFT_DIRECTION_VALUE 	(DIRECTION_DEAD_RANGE_MAX + 0.1)
#define FWD_THROTTLE_VALUE 		THROTTLE_DEAD_RANGE_MAX + 0.1
#define REV_THROTTLE_VALUE 		THROTTLE_DEAD_RANGE_MIN - 0.1

	TEST_F(AOSPTests, front_right) {
		AOSP_ComputeDiffDriveMotorSpeeds(FWD_THROTTLE_VALUE, RGHT_DIRECTION_VALUE, DEFAULT_SPEED, speeds);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_LEFT_MOTOR_PORT],   ABS_MAX_SPEED);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_RGHT_MOTOR_PORT], -ABS_MAX_SPEED * TURNING_DIFF_RATIO);

		AOSP_RunDiffDriveMotors(speeds);
		ASSERT_MOTOR_DIRECTION_PINS(FWD, REV);
	}

	TEST_F(AOSPTests, front_left) {
		AOSP_ComputeDiffDriveMotorSpeeds(FWD_THROTTLE_VALUE, LEFT_DIRECTION_VALUE, DEFAULT_SPEED, speeds);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_LEFT_MOTOR_PORT],  -ABS_MAX_SPEED * TURNING_DIFF_RATIO);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_RGHT_MOTOR_PORT],  ABS_MAX_SPEED);

		AOSP_RunDiffDriveMotors(speeds);
		ASSERT_MOTOR_DIRECTION_PINS(REV, FWD);
	}

	TEST_F(AOSPTests, reverse_right) {
		AOSP_ComputeDiffDriveMotorSpeeds(REV_THROTTLE_VALUE, RGHT_DIRECTION_VALUE, DEFAULT_SPEED, speeds);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_LEFT_MOTOR_PORT],  -ABS_MAX_SPEED);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_RGHT_MOTOR_PORT],  ABS_MAX_SPEED * TURNING_DIFF_RATIO);

		AOSP_RunDiffDriveMotors(speeds);
		ASSERT_MOTOR_DIRECTION_PINS(REV, FWD);
	}

	TEST_F(AOSPTests, reverse_left) {
		AOSP_ComputeDiffDriveMotorSpeeds(REV_THROTTLE_VALUE, LEFT_DIRECTION_VALUE, DEFAULT_SPEED, speeds);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_LEFT_MOTOR_PORT],   ABS_MAX_SPEED * TURNING_DIFF_RATIO);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_RGHT_MOTOR_PORT], -ABS_MAX_SPEED);

		AOSP_RunDiffDriveMotors(speeds);
		ASSERT_MOTOR_DIRECTION_PINS(FWD, REV);
	}

}
bool AOSPTests::bsp_setup_done = false;
