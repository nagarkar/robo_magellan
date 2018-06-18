#include "gtest/gtest.h"
#include "FileUnderTest.h"
#include "madgwick_mahony_common.h"


#define INV_SQRT_ABS_ERR .01f

namespace {
  class MadgwickTest : public ::testing::Test {
  protected:
  };

  TEST_F(MadgwickTest, invSqrt) {
	  ASSERT_NEAR(invSqrt(4.0f), 1/2.0f, INV_SQRT_ABS_ERR);
	  ASSERT_NEAR(invSqrt(16.0f), 1/4.0f, INV_SQRT_ABS_ERR);
  }


}
