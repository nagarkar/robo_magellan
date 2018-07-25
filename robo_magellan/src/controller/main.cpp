#include "qpcpp.h"
#include "bsp.h"
#include "active_objects.h"
#include "util/CircularBuffer.h"
#include "Attitude.h"
#include <unistd.h>
#include <signal.h>

extern "C"
{
	#include <roboticscape.h>
}

using namespace std;

void * runTServer(void * ptr);

void setOptions(int argn, char **argc) {
	int c;
	while ((c = getopt(argn, argc, "s::d::")) != -1) {
		switch (c)
		{
		case 's':
			quaternion_calculation_mode = SOFTWARE;
			break;
		case 'd':
			quaternion_calculation_mode = DMP;
			break;
		default:
			cout << "Unknown option" << endl;
			abort();
		}
	}
}

void bsp_cleanup(int sig) {
	cout << "SIGNAL Received: " << sig << "\n";
	rc_set_state(EXITING);
	BSP_Cleanup();
	exit(0);
}

int main(int argn, char ** argc) {
	rc_enable_signal_handler();
	quaternion_calculation_mode = SOFTWARE;
	// Event queue storage
    static QEvt const *blinkyQSto[10];
	static QEvt const *ahrsQSto[10];
	static QEvt const *gpsQSto[10];
	static QEvt const *dsmQSto[10];
	//static QEvt const *tserverQSto[10];

    BSP_Setup(true); // initialize the Board Support Package
    QF::init(); // initialize the framework and the underlying RT kernel

    // publish-subscribe not used, no call to QF::psInit()
    // dynamic event allocation not used, no call to QF::poolInit()

    // instantiate and start the active objects...

	/*
	AO_TServer->start(3U,					// priority (must be unique)
		tserverQSto, Q_DIM(tserverQSto),			// event queue
		(void *)0, 0U);								// stack (unused)
	*/

	AO_Blinky->start(1U,                            // priority (must be unique)
                     blinkyQSto, Q_DIM(blinkyQSto), // event queue
                     (void *)0, 0U);                // stack (unused)

	AO_AHRS->start(2U,								// priority (must be unique)
					ahrsQSto, Q_DIM(ahrsQSto),		// event queue
					(void *)0, 0U);					// stack (unused)

//	AO_GPS->start(3U,								// priority (must be unique)
//					gpsQSto, Q_DIM(gpsQSto),		// event queue
//					(void *)0, 0U);					// stack (unused)

	AO_DSM->start(4U,								// priority (must be unique)
					dsmQSto, Q_DIM(dsmQSto),		// event queue
					(void *)0, 0U);					// stack (unused)

	// Create a thread (https://goo.gl/ybJAeM) to run grpc.
	pthread_t tserver;
	int tserver_ret = pthread_create(&tserver, NULL, runTServer, (void*)attitudeBuffer);
	cout << "Server Started" << tserver_ret << endl;
    return QF::run(); // run the QF application
}

void * runTServer(void * ptr) {
	CircularBuffer<Attitude> * buffer = (CircularBuffer<Attitude> *)ptr;
	while (true) {
		TelemetryServiceImpl::RunServer(buffer, true /* wait */);
	}
	return 0;
}
