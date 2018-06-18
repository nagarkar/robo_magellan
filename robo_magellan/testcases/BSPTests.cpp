#include "gtest/gtest.h"
#include "FileUnderTest.h"
#include "controller/bsp.h"
#include <gps.h>
#include "roboticscape.h"


#define INV_SQRT_ABS_ERR .01f

namespace {
  class BSPTests : public ::testing::Test {
  protected:
  };

  TEST_F(BSPTests, ledon) {
	  BSP_ledOn();
	  ASSERT_EQ(rc_led_get(RC_LED_GREEN), 1);
  }

// TODO: There is a bug in rc_led_get which fails to return '0' if led is Off (returns 1 instead).
//  TEST_F(BSPTests, ledoff) {
//	  BSP_ledOff();
//	  ASSERT_EQ(rc_led_get(RC_LED_GREEN), 0);
//  }

  TEST_F(BSPTests, setup_gps) {
	  ASSERT_EQ(BSP_GPS_Setup(), BSP_SUCCESS);
  }

  TEST_F(BSPTests, close_gps) {
	  ASSERT_EQ(BSP_GPS_Setup(), BSP_SUCCESS);
	  ASSERT_EQ(BSP_GPS_Close(), BSP_SUCCESS);
  }

//  TEST_F(BSPTests, get_gps) {
//	  gps_data_t gps_out;
//	  BSP_RESULT result = BSP_GPS_SingleShot(&gps_out);
//	  ASSERT_EQ(result, BSP_SUCCESS);
//	  ASSERT_EQ(gps_out.set && LATLON_SET, LATLON_SET);
//	  ASSERT_EQ(gps_out.status, STATUS_FIX);
//  }
  TEST_F(BSPTests, get_gps) {
	  ASSERT_EQ(BSP_GPS_Setup(), BSP_SUCCESS);
	  gps_data_t gps_out;
	  BSP_RESULT res = BSP_GPS_Get(gps_out);
	  ASSERT_EQ(BSP_GPS_Close(), BSP_SUCCESS);

	  // Now check results of the get (after attempting to close)
	  ASSERT_EQ(res, BSP_SUCCESS);
	  ASSERT_EQ(gps_out.set & LATLON_SET, LATLON_SET);
	  ASSERT_EQ(gps_out.status, STATUS_FIX);
  }

}
