#include "gtest/gtest.h"
#include "controller/bsp.h"
#include "roboticscape.h"


#define INV_SQRT_ABS_ERR .01f

#define EXPECTED_CHANNELS 6
#define EXPECTED_RESOLUTION 1024

using namespace testing;
using namespace std;

static bool dsm_data_callback_no_data_called_once = false;
static void dsm_data_callback_no_data(){
	int n_channels = rc_dsm_channels();
	ASSERT_EQ(n_channels, 			EXPECTED_CHANNELS);
	ASSERT_EQ(rc_dsm_resolution(), 	EXPECTED_RESOLUTION);
	dsm_data_callback_no_data_called_once = true;
}

static bool dsm_data_callback_expect_data_channel_1_called_once = false;
static void dsm_data_callback_expect_data_channel_1() {
	int ch_value = rc_dsm_ch_raw(1);
	cout << "Got value on channel 1:" << ch_value << endl;
	if (ch_value > 0) {
		dsm_data_callback_expect_data_channel_1_called_once = true;
	}
}

namespace {
  class DSMTests : public Test {
  protected:
	  void SetUp() {
		  ASSERT_EQ(BSP_DSM_Setup(), BSP_SUCCESS);
	  }

	  void TearDown() {
		  ASSERT_EQ(BSP_DSM_Close(), BSP_SUCCESS);
	  }
  };

  TEST_F(DSMTests, init_close) {
	  ASSERT_EQ(BSP_DSM_Close(), BSP_SUCCESS);
	  ASSERT_EQ(BSP_DSM_Setup(), BSP_SUCCESS);
  }

  TEST_F(DSMTests, downtime) {
	  // Downtime in seconds should be zero since we just enabled the DSM
	  ASSERT_EQ(BSP_DSM_Downtime()/(1000*1000*1000), 0);

	  // Downtime in microseconds should be less than 1 (this can vary depending on architecture)
	  ASSERT_LT(BSP_DSM_Downtime()/(1000), 1);
  }

  /**
   * For this test, make sure there is an active connection
   */
  TEST_F(DSMTests, active_connection) {
	  if (!BSP_DSM_IsUp()) {
		  cout << "WARNING: Could not run active connection tests since DSM connection is not active. Make sure DSM is up and running, paired etc" << endl;
		  return;
	  }
	  ASSERT_TRUE(BSP_DSM_IsUp());
	  ASSERT_EQ(BSP_DSM_Callback(dsm_data_callback_no_data), BSP_SUCCESS);
	  BSP_USLEEP(1*1000*1000);
	  ASSERT_TRUE(dsm_data_callback_no_data_called_once);
  }

  /**
   * For this test, make sure there is an active connection and you are changing data on channel 1 (left joystick on Orange T-Six) within about 1 minute after this test starts.
   */
  TEST_F(DSMTests, active_connection_callbacks) {
	  if (!BSP_DSM_IsUp()) {
		  cout << "WARNING: Could not run active connection tests since DSM connection is not active. Make sure DSM is up and running, paired etc" << endl;
		  return;
	  }
	  rc_dsm_set_callback(dsm_data_callback_expect_data_channel_1);
	  cout << "Start sleeping at us since boot:" << rc_nanos_since_boot()/(1000)	<< endl;
	  // We expect to receive telemetry every 20 ms, so sleep for just more than that.
	  BSP_USLEEP(30*1000);
	  cout << "Done sleeping at us since boot:" << rc_nanos_since_boot()/(1000)	<< endl;
	  ASSERT_TRUE(dsm_data_callback_expect_data_channel_1_called_once);
  }
}
