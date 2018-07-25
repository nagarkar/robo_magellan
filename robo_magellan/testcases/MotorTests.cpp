#include "gtest/gtest.h"
#include "controller/bsp.h"
#include "roboticscape.h"


#define INV_SQRT_ABS_ERR .01f

#define EXPECTED_CHANNELS 6
#define EXPECTED_RESOLUTION 1024

using namespace testing;
using namespace std;

namespace {
  class MotorTests : public Test {
  protected:
	  void SetUp() {
		  BSP_MOTORS_Setup();
	  }

	  void TearDown() {
		  BSP_MOTORS_Close();
	  }
  };


  TEST_F(MotorTests, init_close) {
	  ASSERT_EQ(BSP_MOTORS_Close(), BSP_SUCCESS);
	  ASSERT_EQ(BSP_MOTORS_Setup(), BSP_SUCCESS);
  }

  /**
   * For this test, make sure there is an active connection
   */
  TEST_F(MotorTests, run_motors) {
	  if (!BSP_MOTORS_IsUp()) {
		  cout << "WARNING: Could not run motor tests since Motors are not active." << endl;
		  return;
	  }
	  cout << "Starting Left motor test" << endl;
	  ASSERT_TRUE(BSP_MOTORS_IsUp());
	  ASSERT_EQ(BSP_MOTORS_Set(LEFT_MOTORS, .5), BSP_SUCCESS);
	  sleep(5);
	  ASSERT_EQ(BSP_MOTORS_Set(LEFT_MOTORS, -.5), BSP_SUCCESS);
	  sleep(5);
	  ASSERT_EQ(BSP_MOTORS_Set(LEFT_MOTORS, 0), BSP_SUCCESS);
	  cout << "Starting Right motor test" << endl;
	  ASSERT_EQ(BSP_MOTORS_Set(RIGHT_MOTORS, .5), BSP_SUCCESS);
	  sleep(5);
	  ASSERT_EQ(BSP_MOTORS_Set(RIGHT_MOTORS, -.5), BSP_SUCCESS);
	  sleep(5);
	  ASSERT_EQ(BSP_MOTORS_Set(RIGHT_MOTORS, 0), BSP_SUCCESS);
  }
}
