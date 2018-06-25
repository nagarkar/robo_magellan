#include "gtest/gtest.h"
#include "controller/bsp.h"
#include "roboticscape.h"
#include "controller/aosp.h"

#define INV_SQRT_ABS_ERR .01f

#define EXPECTED_CHANNELS 6
#define EXPECTED_RESOLUTION 1024

using namespace testing;
using namespace std;

namespace {
	class AOSPTests : public Test {
	protected:
		void SetUp() {}
		void TearDown() {}
	};

	TEST_F(AOSPTests, dead_range_max_throttle) {
		double speeds[2];
		GetDiffDriveMotorSpeeds(THROTTLE_DEAD_RANGE_MAX, THROTTLE_DEAD_RANGE_MAX, speeds);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_LEFT_MOTOR_PORT], 0);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_RIGHT_MOTOR_PORT], 0);
	}

	TEST_F(AOSPTests, dead_range_min_throttle) {
		double speeds[2];
		GetDiffDriveMotorSpeeds(THROTTLE_DEAD_RANGE_MIN, THROTTLE_DEAD_RANGE_MIN, speeds);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_LEFT_MOTOR_PORT], 0);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_RIGHT_MOTOR_PORT], 0);
	}

	TEST_F(AOSPTests, forward) {
		double speeds[2];
		GetDiffDriveMotorSpeeds(THROTTLE_DEAD_RANGE_MAX + 0.1, THROTTLE_DEAD_RANGE_MIN, speeds);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_LEFT_MOTOR_PORT], ABS_MAX_SPEED);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_RIGHT_MOTOR_PORT], ABS_MAX_SPEED);
	}

	TEST_F(AOSPTests, reverse) {
		double speeds[2];
		GetDiffDriveMotorSpeeds(THROTTLE_DEAD_RANGE_MIN - 0.1, THROTTLE_DEAD_RANGE_MIN, speeds);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_LEFT_MOTOR_PORT], -1 * ABS_MAX_SPEED);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_RIGHT_MOTOR_PORT],-1 * ABS_MAX_SPEED);
	}

	TEST_F(AOSPTests, front_right) {
		double speeds[2];
		GetDiffDriveMotorSpeeds(THROTTLE_DEAD_RANGE_MAX + 0.1, THROTTLE_DEAD_RANGE_MAX + 0.1, speeds);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_LEFT_MOTOR_PORT],  1 * ABS_MAX_SPEED);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_RIGHT_MOTOR_PORT], 0);
	}

	TEST_F(AOSPTests, front_left) {
		double speeds[2];
		GetDiffDriveMotorSpeeds(THROTTLE_DEAD_RANGE_MAX + 0.1, THROTTLE_DEAD_RANGE_MIN - 0.1, speeds);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_LEFT_MOTOR_PORT],  0);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_RIGHT_MOTOR_PORT], 1 * ABS_MAX_SPEED);
	}

	TEST_F(AOSPTests, reverse_right) {
		double speeds[2];
		GetDiffDriveMotorSpeeds(THROTTLE_DEAD_RANGE_MIN - 0.1, THROTTLE_DEAD_RANGE_MAX + 0.1, speeds);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_LEFT_MOTOR_PORT], -1 * ABS_MAX_SPEED);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_RIGHT_MOTOR_PORT], 0);
	}

	TEST_F(AOSPTests, reverse_left) {
		double speeds[2];
		GetDiffDriveMotorSpeeds(THROTTLE_DEAD_RANGE_MIN - 0.1, THROTTLE_DEAD_RANGE_MIN - 0.1, speeds);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_LEFT_MOTOR_PORT],  0);
		ASSERT_FLOAT_EQ(speeds[DIFF_DRIVE_RIGHT_MOTOR_PORT],-1 * ABS_MAX_SPEED);
	}

}
