#pragma once

#undef USE_GPSD_SHARED_MEMORY

#undef QP_SCAN_INPUT

#include <gps.h>
#include "util/CircularBuffer.h"
#include "Attitude.h"
#include <stdexcept>

uint32_t const QP_CLOCK_PERIOD_MS			= static_cast<uint32_t>(1);
uint32_t const BSP_TICKS_PER_SEC			= QP_CLOCK_PERIOD_MS * 1000;	/* 1 sec = 1000 ms */

uint32_t const AHRS_CLOCK_PERIOD_MS			= static_cast<uint32_t>(100);	/* We want to measure IMU every 10 ms*/
uint32_t const AHRS_TICKS					= AHRS_CLOCK_PERIOD_MS/QP_CLOCK_PERIOD_MS;

uint32_t const BLINKY_CLOCK_PERIOD_MS		= static_cast<uint32_t>(1000);	/* We want to blink every sec */
uint32_t const BLINKY_TICKS					= BLINKY_CLOCK_PERIOD_MS / QP_CLOCK_PERIOD_MS;

typedef enum {
	DMP,
	SOFTWARE
} IMU_MODE;

enum {
	MOTOR_STANDBY_DISABLE = 0,
	MOTOR_STANDBY_ENABLE = 1
};

typedef enum {
	ALL_MOTORS = 0,
	MOTOR_1 = 1,
	MOTOR_2 = 2,
	MOTOR_3 = 3,
	MOTOR_4 = 4
} BSP_MOTOR_ID;

extern IMU_MODE quaternion_calculation_mode;
extern CircularBuffer<Attitude> * attitudeBuffer;

/* Build time exception checking (https://goo.gl/mUpFVg) */
class BSPNotImplementedException : public std::logic_error
{
public:
	BSPNotImplementedException (const string& what_arg): logic_error(what_arg) {}
    virtual const char * what() const noexcept override { return "BSP Function not yet implemented."; }
};


typedef enum {
	BSP_FAILURE,
	BSP_SUCCESS
} BSP_RESULT;

typedef enum {
	ACC,
	GYRO,
	MAG,
	QUAT,
	DMPQUAT,
	STOP
} OUTPUT_MODE;

extern OUTPUT_MODE default_output_mode;

# define BSP_USLEEP(us) { rc_usleep(us); }
# define BSP_Warn(res, warning) { \
	if (res == BSP_FAILURE) { \
		cout << warning << endl; \
	} \
}

BSP_RESULT BSP_Setup(void);
void BSP_ledOff(void);
void BSP_ledOn(void);
void BSP_PublishAttitude(void);
void BSP_SetupIMU(void);
void BSP_Cleanup(void);

void BSP_Toggle_a(void);
void BSP_Toggle_g(void);
void BSP_Toggle_m(void);
void BSP_Toggle_q(void);
void BSP_Toggle_d(void);
void BSP_Toggle_s(void);

BSP_RESULT BSP_GPS_SingleShot(gps_data_t *gps_out);
BSP_RESULT BSP_GPS_Setup();
BSP_RESULT BSP_GPS_Close();
BSP_RESULT BSP_GPS_Get(gps_data_t &gps_out);

BSP_RESULT BSP_DSM_Setup();
BSP_RESULT BSP_DSM_Close();
bool	   BSP_DSM_IsUp();
int64_t    BSP_DSM_Downtime();
BSP_RESULT BSP_DSM_Callback(void (*callback)(void));
double 	   BSP_DSM_Get(int ch);

BSP_RESULT BSP_MOTORS_Setup();
BSP_RESULT BSP_MOTORS_Close();
bool	   BSP_MOTORS_IsUp();
int64_t    BSP_MOTORS_Downtime();
BSP_RESULT BSP_MOTORS_Callback(void (*callback)(void));
BSP_RESULT BSP_MOTORS_Set(BSP_MOTOR_ID motor, double duty_cycle);


uint64_t BSP_millis(void);
uint64_t BSP_nanos(void);

