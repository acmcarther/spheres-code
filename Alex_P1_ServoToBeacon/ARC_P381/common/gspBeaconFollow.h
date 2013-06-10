#ifndef __SPHERES_GSPBEACONFOLLOW__
#define __SPHERES_GSPBEACONFOLLOW__

#ifdef __cplusplus
extern "C" {
#endif


/***********************************/
/*  Begin user-modifiable section  */
/***********************************/
#include "spheres_types.h"
#include "pads.h"

extern dbg_short_packet DebugVec;

/*	Function Name:	gspTaskRun_BeaconFollow()
 *
 *	Input:			The event identifier that called this function
 *					Any extra information needed to resolve the task
 *
 *	Purpose:		Handle any relevant tasks for the sphere	
 *
 */
void gspTaskRun_BeaconFollow(unsigned int gsp_task_trigger, unsigned int extra_data);

/*	Function Name:	gspInitTest_BeaconFollow()
 *
 *	Input:			Number indicating a subtest to execute
 *
 *	Purpose:		Initialize the custom functionality of this test
 *
 */
void gspInitTest_BeaconFollow(unsigned int test_number);

/*	Function Name:	gspControl_BeaconFollow()
 *
 *	Input:			Test initialization time
 *					Identifier for the maneuver					
 *					Maneuver run time
 *
 *	Purpose:		Execute the control sequences for the sphere	
 *
 */
void gspControl_BeaconFollow(unsigned int test_number, unsigned int test_time,
	unsigned int maneuver_number, unsigned int maneuver_time);

/*	Function Name:	gspPadsInertial_BeaconFollow()
 *
 *	Input:			Data samples of the internal gyro
 *					Total number of samples
 *
 *	Purpose:		Implement some action based on new inertial data
 *
 */
void gspPadsInertial_BeaconFollow(IMU_sample *accel, IMU_sample *gyro, 
	unsigned int num_samples);

/*	Function Name:	gspPadsGlobal_BeaconFollow()
 *
 *	Input:			Beacon identifier
 *					Measurements indicating distance and direction to beacon
 *
 *	Purpose:		If the calling beacon is the remote beacon, we'll update our remote information
 *
 */
void gspPadsGlobal_BeaconFollow(unsigned int beacon, beacon_measurement_matrix measurements);

/*	Function Name:	gspSetTarget_BeaconFollow()
 *
 *
 *	Purpose:		
 *
 */
void gspSetTarget_BeaconFollow(unsigned int test_number, unsigned int maneuver_time, 
	float *ctrlStateTarget, unsigned int *maneuver_timeout);

void gspInitProgram_BeaconFollow();


/***********************************/
/*   End user-modifiable section   */
/***********************************/

#ifdef __cplusplus
}
#endif

#endif
