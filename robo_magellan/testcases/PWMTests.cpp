#include <gtest/gtest.h>
#include "controller/bsp.h"
#include "roboticscape.h"


#define INV_SQRT_ABS_ERR .01f

#define EXPECTED_CHANNELS 6
#define EXPECTED_RESOLUTION 1024

#define SECS_REQUIRED_TO_MANUALLY_VERIFY_PWM_ON_SCOPE 1

using namespace testing;
using namespace std;

namespace {
  class PWMTests : public Test {
  protected:
	  void SetUp() {
		  BSP_PWM_Setup();
	  }

	  void TearDown() {
		  BSP_PWM_Close();
	  }
  };

  TEST_F(PWMTests, init_close) {
	  ASSERT_EQ(BSP_PWM_Close(), BSP_SUCCESS);
	  ASSERT_EQ(BSP_PWM_Setup(), BSP_SUCCESS);
  }


  TEST_F(PWMTests, run_pwm) {
	  ASSERT_EQ(BSP_PWM_Set(PWM_CHANNEL_A, 0.8f), BSP_SUCCESS);
	  ASSERT_EQ(BSP_PWM_Set(PWM_CHANNEL_B, 0.1f), BSP_SUCCESS);
	  sleep(SECS_REQUIRED_TO_MANUALLY_VERIFY_PWM_ON_SCOPE);
  }

}
