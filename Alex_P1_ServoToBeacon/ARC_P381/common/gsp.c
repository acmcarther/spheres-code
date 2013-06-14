/* 
 * gsp.c
 *
 * SPHERES Guest Scientist Program custom source code template.
 *
 * MIT Space Systems Laboratory
 * SPHERES Guest Scientist Program
 * http://ssl.mit.edu/spheres/
 * 
 * Copyright 2005 Massachusetts Institute of Technology
 */


/*----------------------------------------------------------------------------*/
/*                         Do not modify this section.                        */
/*----------------------------------------------------------------------------*/
#include "ctrl_attitude.h"
#include "ctrl_position.h"
#include "find_state_error.h"
#include "ctrl_mix.h"
#include "gsutil_checkout.h"

#include "comm.h"
#include "comm_internal.h"
#include "commands.h"
#include "comm_process_rx_packet.h"
#include "control.h"
#include "gsp.h"
#include "gsp_task.h"
#include "housekeeping.h"
#include "pads.h"
#include "prop.h"
#include "spheres_constants.h"
#include "spheres_physical_parameters.h"
#include "spheres_types.h"
#include "std_includes.h"
#include "system.h"
#include "util_memory.h"
#include "smt335async.h"
#include "exp_v2.h"
#include <string.h>

/*----------------------------------------------------------------------------*/
/*                     Modify as desired below this point.                    */
/*----------------------------------------------------------------------------*/
#include "smartphone_comm_utils.h"
#include "math.h"

#include "pads_internal.h"
//#include "gsutil_PadsImu.h"

//#include "comm_datacomm.h"
//#include "gsutil_PadsUS.h"

#include "gspBeaconFollow.h"
#include "gspIpadHolder.h"
#include "gspAutoCamera.h"
 
// pick ONE
#define LAB_VERSION
//#define ISS_VERSION

#ifdef ISS_VERSION
#define DEFAULT_X 0.0
#define PROGRAM_NUMBER 999 // Make a proper number for this
#else //LAB_VERSION
#define DEFAULT_X -0.67
#define PROGRAM_NUMBER 999 // Make a proper number for this
#endif

static int testClass;

#define CHECKOUT_TESTNUM_OFFSET			(000)
#define BEACON_FOLLOW_TESTNUM_OFFSET	(200)
#define IPAD_HOLDER_TESTNUM_OFFSET		(300)
#define AUTO_CAMERA_TESTNUM_OFFSET		(400)

const unsigned int refTestNumber[] = {1,201,301,401};
static unsigned int testNum = 201;

typedef enum _test_class {invalid_test, checkout_tests, beacon_follow_test,
	ipad_holder_test, auto_camera_test} test_class;

test_class getTestClass(unsigned int test_num)
{
	if ((test_num > CHECKOUT_TESTNUM_OFFSET) && (test_num <= CHECKOUT_TESTNUM_OFFSET + 100))
		return checkout_tests;

	if ((test_num > BEACON_FOLLOW_TESTNUM_OFFSET) && (test_num <= BEACON_FOLLOW_TESTNUM_OFFSET + 100))
		return beacon_follow_test;

	if ((test_num > IPAD_HOLDER_TESTNUM_OFFSET) && (test_num <= IPAD_HOLDER_TESTNUM_OFFSET + 100))
		return ipad_holder_test;	

	if ((test_num > AUTO_CAMERA_TESTNUM_OFFSET) && (test_num <= AUTO_CAMERA_TESTNUM_OFFSET + 100))
		return auto_camera_test;

	return invalid_test;
}

// callback function prototype
void gspProcessPhoneCommand(unsigned char channel, unsigned char* buffer, unsigned int len);

/* Function Name:	gspIdentitySet()
 * 
 * Purpose:			Sets the identifier of the sphere.
 *
 */
void gspIdentitySet()
{
	// set the logical identifier (SPHERE#) for this vehicle
	sysIdentitySet(SPHERE_ID);
}

/* Function Name:	gspInitProgram()
 * 
 * Purpose:			Initializes comms and major systems of the sphere
 *
 */
void gspInitProgram()
{
	// set the unique program identifier (to be assigned by MIT)
	sysProgramIDSet(PROGRAM_NUMBER);

	// set up communications TDMA frames
	commTdmaStandardInit(COMM_CHANNEL_STL, sysIdentityGet(), NUM_SPHERES);
	commTdmaStandardInit(COMM_CHANNEL_STS, sysIdentityGet(), NUM_SPHERES);
	commTdmaStandardInit(COMM_CHANNEL_EXP, sysIdentityGet(), NUM_SPHERES);

	// enable communications channels
	commTdmaEnable(COMM_CHANNEL_STL);
	commTdmaEnable(COMM_CHANNEL_EXP);
   
	// allocate storage space for IMU samples
	padsInertialAllocateBuffers(50);

	// inform system of highest beacon number in use
	padsInitializeFPGA(NUM_BEACONS);

	/* custom program initialization goes below this point */	

	// set up global and inertial sensors
	padsInertialPeriodSet(SYS_FOREVER,SYS_FOREVER);
	padsGlobalPeriodSet(SYS_FOREVER);

	// turn off background telemetry by default
	commBackgroundTelemetryPeriodSet(SYS_FOREVER);

	// TODO: turn off onboard beacon
	padsBeaconNumberSet(0);

}

/* Function Name:	gspInitTest()
 * 
 * Input:			Test number identifying a specific set of maneuvers
 * Purpose:			Initializes comms and major systems of the sphere
 *
 */
void gspInitTest(unsigned int test_number)
{
	// Define function variables
	unsigned int index;

	// Determine test class and number from given information
	index = test_number-1;
	testClass = getTestClass(refTestNumber[index]);
	testNum = refTestNumber[index] % 100;

	// Run initialization tasks determined by test
	//		TODO: Build the subfunctions required for this
	switch(testClass)
	{
		case(checkout_tests):
			//gspInitTest_Checkout(testNum);
			break;
		case(beacon_follow_test):
			gspInitTest_BeaconFollow(testNum);
			break;
		case(ipad_holder_test):
			//gspInitTest_IpadHolder(testNum);
			break;
		case(auto_camera_test):
			//gspInitTest_AutoCamera(testNum);
			break;
		default:
			ctrlTestTerminate(TEST_RESULT_UNKNOWN_TEST);
			break;
	}

}

/* Function Name: 	gspInitTask()
 * 
 * Purpose:			Set the triggers for event driven code segments.	
 *
 */
void gspInitTask() 
{
	// Task trigger on finished estimator or new global beacon data
	//		TODO: Ensure that we do not need any more events
	taskTriggerMaskSet(PADS_GLOBAL_BEACON_TRIG|PADS_INERTIAL_TRIG|TEST_START_TRIG);
}

/* Function Name: 	gspPadsInertial()
 * 
 * Input:			Data samples indicating positional displacement
 *					Data samples indicating angular displacement
 *					Counter identifying the total number of samples
 *				
 * Purpose:			Reacts to new inertial data from the system typically by adjusting
 *					the target state or movement parameters.
 *
 */
void gspPadsInertial(IMU_sample *accel, IMU_sample *gyro, unsigned int num_samples)
{
	/* NOTICE: This function misbehaves if you run it before gspInitTest */

	// Run inertial update routines for the specific test classes
	//		TODO: Build the subfunctions required for this
	switch(testClass)
	{
		case(checkout_tests):
			//gspPadsInertial_Checkout(accel, gyro, num_samples);
			break;
		case(beacon_follow_test):
			gspPadsInertial_BeaconFollow(accel, gyro, num_samples);
			break;
		case(ipad_holder_test):
			//gspPadsInertial_IpadHolder(accel, gyro, num_samples);
			break;
		case(auto_camera_test):
			//gspPadsInertial_AutoCamera(accel, gyro, num_samples);
			break;
		default:
			break;
	}
}

/* Function Name: 	gspPadsGlobal()
 * 
 * Input:			Index identifying a beacon
 *					Data samples containing measurements for the beacon
 *				
 * Purpose:			Reacts to new global data from the system, typically by adjusting
 *					the target state or movement parameters.
 *
 */
void gspPadsGlobal(unsigned int beacon, beacon_measurement_matrix measurements)
{
	/* NOTICE: This function misbehaves if you run it before gspInitTest */

	// Run global update routines for the specific test classes
	//		TODO: Build the subfunctions required for this
	switch(testClass)
	{
		case(checkout_tests):
			//gspPadsGlobal_Checkout(beacon, measurements);
			break;
		case(beacon_follow_test):
			gspPadsGlobal_BeaconFollow(beacon, measurements);
			break;
		case(ipad_holder_test):
			//gspPadsGlobal_IpadHolder(beacon, measurements);
			break;
		case(auto_camera_test):
			//gspPadsGlobal_AutoCamera(beacon, measurements);
			break;
		default:
			break;
	}
}

/* Function Name: 	gspTaskRun()
 * 
 * Input:			Number representing the event that triggered the function call.
 *					Additional data for the task response function.
 *				
 * Purpose:			Delegates task responses based on events that fired.
 *
 */
void gspTaskRun(unsigned int gsp_task_trigger, unsigned int extra_data) 
{
	switch(testClass)
	{
		case(checkout_tests):
			gspTaskRun_Checkout(gsp_task_trigger, extra_data);
			break;
		case(beacon_follow_test):
			gspTaskRun_BeaconFollow(gsp_task_trigger, extra_data);
			break;
		case(ipad_holder_test):
			gspTaskRun_IpadHolder(gsp_task_trigger, extra_data);
			break;
		case(auto_camera_test):
			gspTaskRun_AutoCamera(gsp_task_trigger, extra_data);
			break;
		default:
			break;
	}
}

/* Function Name: 	gspControl()
 * 
 * Input:			Test number identifying a specific set of maneuvers
 *					Time identifying execution period
 *					Index indicating which maneuver to execute
 *					Time indicating maneuver period
 *				
 * Purpose:			Executes control sequences specific to particular tests
 *
 */
void gspControl(unsigned int test_number, unsigned int test_time, unsigned int maneuver_number, 
	unsigned int maneuver_time)
{	
	/* NOTICE: This function misbehaves if you run it before gspInitTest */

	// Run control routines for the specific test classes
	//		TODO: Build the subfunctions required for this
	switch(testClass)
	{
		case(checkout_tests):
			gspControl_Checkout(testNum, test_time, maneuver_number, maneuver_time);
			break;
		case(beacon_follow_test):
			gspControl_BeaconFollow(testNum, test_time, maneuver_number, maneuver_time);
			break;
		case(ipad_holder_test):
			gspControl_IpadHolder(testNum, test_time, maneuver_number, maneuver_time);
			break;
		case(auto_camera_test):
			gspControl_AutoCamera(testNum, test_time, maneuver_number, maneuver_time);
			break;
		default:
			break;
	}
}

// Unneeded functions required for compilation

int checksumChecks(unsigned char* buffer, unsigned int len) {
	return 0;
}
void gspProcessPhoneCommand(unsigned char channel, unsigned char* buffer, unsigned int len) {}
