#include "gtest/gtest.h"
#include "controller/bsp.h"
#include "roboticscape.h"


#define INV_SQRT_ABS_ERR .01f

#define EXPECTED_CHANNELS 6
#define EXPECTED_RESOLUTION 1024

using namespace testing;
using namespace std;

namespace {
  class EduMotorTests : public Test {
  protected:
	  void SetUp() {
		  BSP_EDUMIP_MOTORS_Setup();
	  }

	  void TearDown() {
		  BSP_EDUMIP_MOTORS_Close();
	  }
  };


  TEST_F(EduMotorTests, init_close) {
	  ASSERT_EQ(BSP_EDUMIP_MOTORS_Close(), BSP_SUCCESS);
	  ASSERT_EQ(BSP_EDUMIP_MOTORS_Setup(), BSP_SUCCESS);
  }

  /**
   * For this test, make sure there is an active connection
   */
  TEST_F(EduMotorTests, run_motors) {
	  if (!BSP_EDUMIP_MOTORS_IsUp()) {
		  cout << "WARNING: Could not run motor tests since Motors are not active." << endl;
		  return;
	  }
	  const int motor = MOTOR_3;
	  ASSERT_TRUE(BSP_EDUMIP_MOTORS_IsUp());
	  ASSERT_EQ(BSP_EDUMIP_MOTORS_Set(motor, .3), BSP_SUCCESS);
	  sleep(60);
	  BSP_USLEEP(10*1000*1000);
	  BSP_EDUMIP_MOTORS_Set(motor, 0);
  }
}
