#include "bsp.h"
#include "util/bsp_helpers.h"

#include "qpcpp.h"
#include <libgpsmm.h>
#include <bbahrs/MahonyAHRS.h>
#include <bbahrs/MadgwickAHRS.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>      // for memcpy() and memset()
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>
#include <telemetry_service.grpc.pb.h>

#include "active_objects.h"
#include "Attitude.h"
#include "TelemetryServiceImpl.h"
#include "util/CircularBuffer.h"

// This extern "C" is required because roboticscape.h does not wrap it's declarations in extern "C"
extern "C"
{
	#include <roboticscape.h>
}

using namespace std;
using namespace telemetry;

Q_DEFINE_THIS_FILE

#ifdef Q_SPY
#error Simple Blinky Application does not provide Spy build configuration
#endif

#define SAMPLE_RATE_HZ 100;

// Private methods
static void updateAttitudeBuffer();
static void imu_interrupt_handler();
static void normalizeQ(Q_cxyz &q);
static void print_options();

IMU_MODE quaternion_calculation_mode(SOFTWARE);

/******* Utilities ***/
BSP_RESULT BSP_RC_RESULT(int result) {
	if (result == -1) {
		return BSP_FAILURE;
	} else {
		return BSP_SUCCESS;
	}
}

/********     GENERAL BOARD SUPPORT   *********/
/**********************************************/

static Server * server;

BSP_RESULT BSP_Setup(void) {
	cout	<< "IMU Streaming Data..." << endl
			<< "QP version: " << QP_VERSION_STR << endl			
			<< "Press ESC to quit..." << endl;
	print_options();

	if(rc_kill_existing_process(2.0 /* timeout sec */) < -2) {
		return BSP_FAILURE;
	}
	rc_make_pid_file(); // Guaranteed to work if rc_kill_existing_process works.


	if(rc_enable_signal_handler() == -1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return BSP_FAILURE;
	}

	return BSP_SUCCESS;
}

void BSP_Cleanup() {
	cout << "doing rc_mpu_power_off" << endl;
	rc_mpu_power_off();
	cout << "doing rc_led_cleanup" << endl;
	rc_led_cleanup();
	if (server != NULL) {
		cout << "doing server->Shutdown()" << endl;
		server->Shutdown();
	}
	cout << "doing BSP_GPS_Close()" << endl;
	BSP_GPS_Close();
}

void print_options() {
	cout << "IMU Calculation Mode:" << (quaternion_calculation_mode == DMP ? "DMP" : "SOFTWARE") << "\n";
	cout << "Options:\n";
	cout << "\t\tESC key" << "Exit" << "\n";
	cout << "\t\ts:" << "Stop data print" << "\n";
	cout << "\t\ta:" << "Show Acc" << "\n";
	cout << "\t\tg:" << "Show Gyro" << "\n";
	cout << "\t\tq:" << "Show software computed quaternion" << "\n";
	cout << "\t\td:" << "Show DMP Quaternion" << "\n";
	cout << "\t\tm:" << "Show Magnetogmeter readings" << "\n";
}

void BSP_Toggle_a(void) { default_output_mode = ACC; }
void BSP_Toggle_g(void) { default_output_mode = GYRO; }
void BSP_Toggle_m(void) { default_output_mode = MAG; }
void BSP_Toggle_q(void) { default_output_mode = QUAT; }
void BSP_Toggle_d(void) { default_output_mode = DMPQUAT; }
void BSP_Toggle_s(void) { default_output_mode = STOP; }

OUTPUT_MODE default_output_mode(QUAT);



/**********	    MOTORS             ************/
/**********************************************/

BSP_RESULT BSP_Motor_Setup(void) {
	if(rc_motor_init()==-1){
 		fprintf(stderr,"ERROR: failed to initialize motors\n");
 		return BSP_FAILURE;
 	}
 	rc_motor_standby(MOTOR_STANDBY_ENABLE); // start with motors in standby

 	return BSP_SUCCESS;
}

//............................................................................
void BSP_ledOff(void) {
	rc_led_set(RC_LED_GREEN, 0);    
}
//............................................................................
void BSP_ledOn(void) {
	rc_led_set(RC_LED_GREEN, 1);
}

static rc_mpu_data_t imu_data;
static rc_mpu_config_t imu_config;
static Q_cxyz complementary_filter = {1.0f, 0.0f, 0.0f, 0.0f};

// < 0 indicates failure.
int BSP_InitProcess() {	
	/*
	if (rc_initialize() < 0) {
		cout << "RC Initialization Failed" << endl;
		return -1;
	}
	*/
	return 0;
}


/* Buffers */
CircularBuffer<struct gps_data_t> gpsBuffer(10);

CircularBuffer<Attitude> l_attitudeBuffer(10);
CircularBuffer<Attitude> * attitudeBuffer = &l_attitudeBuffer; // opaque pointer



/**********	    GPS GPS            ************/
/**********************************************/

struct gps_data_t g_gps_data;

BSP_RESULT BSP_DSM_Setup() {
	if(rc_dsm_init() == -1){
		fprintf(stderr,"failed to start initialize DSM\n");
		return BSP_FAILURE;
	}
	return BSP_SUCCESS;
}

BSP_RESULT BSP_DSM_Close() {
	if(rc_dsm_cleanup() == -1){
		fprintf(stderr,"failed to cleanup initialize DSM\n");
		return BSP_FAILURE;
	}
	return BSP_SUCCESS;
}

bool BSP_DSM_IsUp() {
	if (rc_dsm_is_connection_active() != 0) {
		return true;
	}
	// Wait, checking if new data is present periodically.
	int wait_time_us = 1*1000*1000;
	int loop_time = 50*1000;
	while(rc_dsm_is_new_data()==0 && wait_time_us > 0) {
		BSP_USLEEP(loop_time);
		wait_time_us -= loop_time;
	}
	// Recheck if we are officially up.
	if (rc_dsm_is_connection_active() != 0) {
		return true;
	}
	return false;

}

int64_t BSP_DSM_Downtime() {
	return rc_dsm_nanos_since_last_packet();
}

BSP_RESULT BSP_DSM_Callback(void (*callback)(void)) {
	rc_dsm_set_callback(callback);
	return BSP_SUCCESS;
}

double BSP_DSM_Get(int ch) {
	return rc_dsm_ch_normalized(ch);
}

/****     Motor **********/
static bool motor_setup_done = false;
BSP_RESULT BSP_MOTORS_Setup() {
	BSP_RESULT result = BSP_RC_RESULT(rc_motor_init());
	motor_setup_done = (result == BSP_SUCCESS ? true : false);
	return result;

}

BSP_RESULT BSP_MOTORS_Close() {
	motor_setup_done = false;
	return BSP_RC_RESULT(rc_motor_cleanup());
}

bool BSP_MOTORS_IsUp() {
	return motor_setup_done;
}

int64_t BSP_MOTORS_Downtime() {
	throw BSPNotImplementedException("");
}

BSP_RESULT BSP_MOTORS_Callback(void (*callback)(void)) {
	throw BSPNotImplementedException("");
}

BSP_RESULT BSP_MOTORS_Set(BSP_MOTOR_ID motor, double duty_cycle) {
	return BSP_RC_RESULT(rc_motor_set((int)motor, duty_cycle));
}


/***** GPS ***********/
BSP_RESULT BSP_GPS_SingleShot(gps_data_t *gps_out) {

	gpsmm gps_rec("localhost", DEFAULT_GPSD_PORT);

	if (gps_rec.stream(WATCH_ENABLE|WATCH_JSON) == NULL) {
		cerr << "No GPSD running.\n";
		return BSP_FAILURE;
	}

	for(;;) {
		struct gps_data_t* newdata;
		if (!gps_rec.waiting(500)) {
			continue;
		}
		if ((newdata = gps_rec.read()) == NULL) {
			cerr << "Read error.\n";
			return BSP_FAILURE;
		} else if (newdata->status != STATUS_FIX){
			cout << ".";
			continue;
		}
		*gps_out = *newdata;
		gpsBuffer.put(*newdata);
		return BSP_SUCCESS;
	}
	return BSP_SUCCESS;
}

#ifdef USE_GPSD_SHARED_MEMORY

BSP_RESULT BSP_GPS_Setup() {
	int result;
	struct timeval tv;

	result = gps_open("localhost", GPSD_SHARED_MEMORY, &g_gps_data);
	if (result == -1) {
	    printf("code: %d, reason: %s\n", result, gps_errstr(result));
	    return BSP_FAILURE;
	}
	return BSP_SUCCESS;
}


BSP_RESULT BSP_GPS_Close() {

	if (gps_close (&g_gps_data) == -1) {
		return BSP_FAILURE;
	}
	return BSP_SUCCESS;
}

BSP_RESULT BSP_GPS_Get(gps_data_t &gps_out) {
	BSP_RESULT result = BSP_FAILURE;
	int rc;
	if ((rc = gps_read(&g_gps_data)) == -1) {
		printf("Error occurred reading GPS data. code: %d, reason: %s\n", rc, gps_errstr(rc));
	} else {
		struct gps_fix_t fix = g_gps_data.fix;
		if ((g_gps_data.status == STATUS_FIX) && (fix.mode == MODE_2D || fix.mode == MODE_3D) &&
			!isnan(fix.latitude) && !isnan(fix.longitude)) {
				result = BSP_SUCCESS;
				//printf("latitude: %f, longitude: %f, speed: %f, timestamp: %lf\n", fix.latitude, fix.longitude, fix.speed, fix.time); //EDIT: Replaced tv.tv_sec with gps_data.fix.time
				gps_out = g_gps_data;
				gpsBuffer->put(g_gps_data);
		} else {
			printf("no GPS data available\n");
		}
	}
	return result;
}

#else
BSP_RESULT BSP_GPS_Setup() {
	int result;
	struct timeval tv;

	result = gps_open("localhost", DEFAULT_GPSD_PORT, &g_gps_data);
	if (result == -1) {
	    cout << "code: "<< result << ", reason: %s" << gps_errstr(result) << "\n";
	    return BSP_FAILURE;
	} else {
		result = gps_stream(&g_gps_data, WATCH_ENABLE | WATCH_JSON, NULL);
		if (result == -1) {
			return BSP_FAILURE;
		}
	}
	return BSP_SUCCESS;
}


BSP_RESULT BSP_GPS_Close() {
	if (gps_stream(&g_gps_data, WATCH_DISABLE, NULL) == -1) {
		return BSP_FAILURE;
	}
	if (gps_close(&g_gps_data) == -1) {
		return BSP_FAILURE;
	}
	return BSP_SUCCESS;
}

BSP_RESULT BSP_GPS_Get(gps_data_t &gps_out) {
	int rc;
	bool gotfix = false;
	bool nTrials = 100;
	if (gps_waiting (&g_gps_data, 500)) {
		while(!gotfix && nTrials > 0) {
			if ((rc = gps_read(&g_gps_data)) == -1) {
				cout << "Error occurred reading GPS data. code: "<< rc << ", reason: " << gps_errstr(rc) << "\n";
			} else {
				struct gps_fix_t fix = g_gps_data.fix;
				if ((g_gps_data.status == STATUS_FIX) && (fix.mode == MODE_2D || fix.mode == MODE_3D) &&
					!isnan(fix.latitude) && !isnan(fix.longitude)) {
						gotfix = true;
						cout << "latitude: "<< fix.latitude << ", longitude: "<< fix.longitude << ", speed: "<< fix.speed << ", timestamp: "<< fix.time << "\n"; //EDIT: Replaced tv.tv_sec with gps_data.fix.time
				}
			}
			nTrials++;
		}
	}
	if (gotfix) {
		gps_out = g_gps_data;
		gpsBuffer.put(g_gps_data);
		return BSP_SUCCESS;
	}
	return BSP_FAILURE;
}

#endif


/**********	 INERTIAL MEASUREMENT  ************/
/**********************************************/

static void imu_interrupt_handler() {
	updateAttitudeBuffer();
}


// DMP version
void BSP_SetupIMU_DMP() {	
	cout << "Setting up IMU" << endl;
	imu_config = rc_mpu_default_config();
	imu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	imu_config.enable_magnetometer = 1;	
#ifdef SHOW_WARNINGS
		imu_config.show_warnings = 1;
#endif
	imu_config.orient = ORIENTATION_Y_UP;
	if (rc_mpu_initialize_dmp(&imu_data, imu_config) < 0) {
		fprintf(stderr, "ERROR: can't talk to IMU DMP, all hope is lost\n");
		//return -1;
	}
	if (rc_mpu_set_dmp_callback(&imu_interrupt_handler) < 0) {
		fprintf(stderr, "ERROR: can't setup interrupt handler for imu.\n");
	}
}

void BSP_SetupIMU_Software() {
	sampleFreq = BSP_TICKS_PER_SEC/AHRS_CLOCK_PERIOD_MS;
	beta = beta * 2;
	cout << "Setting up IMU with SOFTWARE configuration" << endl;
	imu_config = rc_mpu_default_config();
	imu_config.enable_magnetometer = 1;
#ifdef SHOW_WARNINGS
	imu_config.show_warnings = 1;
#endif
	imu_config.orient = ORIENTATION_Y_UP;
	if (rc_mpu_initialize(&imu_data, imu_config) < 0 /* zero on success */) {
		cout << "IMU setup failed" << endl;
	} else {
		cout << "IMU Setup done" << endl;
	}	
	if (rc_mpu_set_dmp_callback(&imu_interrupt_handler) < 0) {
		fprintf(stderr, "ERROR: can't setup interrupt handler for imu.\n");
	}	
}

void BSP_SetupIMU(){	
	if (quaternion_calculation_mode == DMP) {
		BSP_SetupIMU_DMP();
	}
	else if (quaternion_calculation_mode == SOFTWARE) {
		BSP_SetupIMU_Software();
	}
}

static void updateAttitudeBuffer() {
	uint64_t timestamp = BSP_nanos(); 
	if (quaternion_calculation_mode == SOFTWARE) {
		float degToRad = 0.0174532925199;
		//for (uint16_t i = 0; i < AHRS_CLOCK_PERIOD_MS; i++) {
			MadgwickAHRSupdate(&complementary_filter,
				imu_data.gyro[0] * degToRad,
				imu_data.gyro[1] * degToRad,
				imu_data.gyro[2] * degToRad,
				imu_data.accel[0],
				imu_data.accel[1],
				imu_data.accel[2],
				imu_data.mag[0],
				imu_data.mag[1],
				imu_data.mag[2]);
		//}

		normalizeQ(complementary_filter);
		attitudeBuffer->put({ complementary_filter.c, complementary_filter.x, complementary_filter.y, complementary_filter.z, timestamp });	
		//cout << "(a)" << flush;
	}
	else if (quaternion_calculation_mode == DMP) {
		float c = imu_data.fused_quat[0];
		float x = imu_data.fused_quat[1];
		float y = imu_data.fused_quat[2];
		float z = imu_data.fused_quat[3];
		float norm = c*c + x*x + y*y + z*z;
		float sqrt_norm = sqrtf(norm);
		c = c / sqrt_norm;
		x = x / sqrt_norm;
		y = y / sqrt_norm;
		z = z / sqrt_norm;		
		attitudeBuffer->put({ c, x, y, z, timestamp });
	}
	
	switch (default_output_mode) {
	case ACC:
		cout << "acc," << imu_data.accel[0] << "," << imu_data.accel[1] << "," << imu_data.accel[2] << endl;		
		break;
	case GYRO:
		cout << "gyro," << imu_data.gyro[0] << "," << imu_data.gyro[1] << "," << imu_data.gyro[2] << endl;
		break;
	case MAG:
		cout << "mag," << imu_data.mag[0] << "," << imu_data.mag[1] << "," << imu_data.mag[2] << endl;
		break;
	case QUAT:
		cout << "orientation," << complementary_filter.c << "," << complementary_filter.x << "," << complementary_filter.y << "," << complementary_filter.z << "," << timestamp << endl;
		break;
	case DMPQUAT:
		cout << "dmp," << imu_data.fused_quat[0]<< "," << imu_data.fused_quat[1] << "," << imu_data.fused_quat[1] << "," << imu_data.fused_quat[2] << "," << timestamp << endl;
		break;
	case STOP:
	default:
		break;
	}
	//cout << "*" << flush;
}

void BSP_PublishAttitude(void) {
	static int mag_skip_counter = 0;
	//cout << "." << flush;
	if (rc_mpu_read_accel(&imu_data) != 0) {
		cout << "can't read acc" << endl;
	}
	if (rc_mpu_read_gyro(&imu_data) != 0) {
		cout << "Can't read gyro" << endl;
	}
	if (mag_skip_counter % 10 == 0) {
		if (rc_mpu_read_mag(&imu_data) != 0) {
			cout << "can't read mag" << endl;
		}		
	}
	updateAttitudeBuffer();
}
