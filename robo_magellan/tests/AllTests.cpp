
#include "gtest/gtest.h"
#include <libgpsmm.h>
#include <signal.h>

#include "SampleTest.cpp"
#include "MadgwickTests.cpp"
//#include "LedGPSTests.cpp"
#include "CircularBufferTests.cpp"
#include "DSMTests.cpp"
#include "MotorTests.cpp"
#include "AOSPTests.cpp"

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

void handle_sigint(int sig)
{
    printf("Caught signal %d\n", sig);
}

int vmain(void)
{
  signal(SIGINT, handle_sigint);
  gpsmm gps_rec("localhost", DEFAULT_GPSD_PORT);

  if (gps_rec.stream(WATCH_ENABLE | WATCH_JSON) == NULL) {
    std::cerr << "No GPSD running.\n";
    return 1;
  }

  for (;;) {
    struct gps_data_t *gpsd_data;

    if (!gps_rec.waiting(500)) continue;

    if ((gpsd_data = gps_rec.read()) == NULL) {
      std::cerr << "GPSD read error.\n";
      return 1;
    } else {
      while (((gpsd_data = gps_rec.read()) == NULL) ||
             (gpsd_data->fix.mode < MODE_2D)) {
    	  usleep(100000);
      }
      timestamp_t ts   = gpsd_data->fix.time;
      double latitude  = gpsd_data->fix.latitude;
      double longitude = gpsd_data->fix.longitude;

      // convert GPSD's timestamp_t into time_t
      time_t seconds;
      seconds = (time_t)ts;
      auto tm = *std::localtime(&seconds);

      std::ostringstream oss;
      oss << std::put_time(&tm, "%d-%m-%Y %H:%M:%S");
      auto time_str = oss.str();

      // set decimal precision
      std::cout.precision(6);
      std::cout.setf(std::ios::fixed, std::ios::floatfield);
      std::cout << time_str << "," <<
        latitude << "," <<
        longitude << '\n';
    }
  }
  return 0;
}
