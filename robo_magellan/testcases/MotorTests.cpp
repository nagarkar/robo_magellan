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
	  const int motor = MOTOR_3;
	  ASSERT_TRUE(BSP_MOTORS_IsUp());
	  ASSERT_EQ(BSP_MOTORS_Set(motor, 0.5), BSP_SUCCESS);
	  BSP_USLEEP(0.1*1000*1000);
	  BSP_MOTORS_Set(motor, 0);
  }
}
