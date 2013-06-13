#ifndef __SPHERES_GSPAUTOCAMERA__
#define __SPHERES_GSPAUTOCAMERA__

#ifdef __cplusplus
extern "C" {
#endif


/***********************************/
/*  Begin user-modifiable section  */
/***********************************/
#include "spheres_types.h"
#include "pads.h"


/*	Function Name:	gspTaskRun_AutoCamera()
 *
 *	Input:			The event identifier that called this function
 *					Any extra information needed to resolve the task
 *
 *	Purpose:		Handle any relevant tasks for the sphere	
 *
 */
void gspTaskRun_AutoCamera(unsigned int gsp_task_trigger, unsigned int extra_data);

/*	Function Name:	gspInitTest_AutoCamera()
 *
 *	Input:			Number indicating a subtest to execute
 *
 *	Purpose:		Initialize the custom functionality of this test
 *
 */
void gspInitTest_AutoCamera(unsigned int test_number);

/*	Function Name:	gspControl_AutoCamera()
 *
 *	Input:			Test initialization time
 *					Identifier for the maneuver					
 *					Maneuver run time
 *
 *	Purpose:		Execute the control sequences for the sphere	
 *
 */
void gspControl_AutoCamera(unsigned int test_number, unsigned int test_time,
	unsigned int maneuver_number, unsigned int maneuver_time);

/*	Function Name:	gspPadsInertial_AutoCamera()
 *
 *	Input:			Data samples of the internal gyro
 *					Total number of samples
 *
 *	Purpose:		Implement some action based on new inertial data
 *
 */
void gspPadsInertial_AutoCamera(IMU_sample *accel, IMU_sample *gyro, 
	unsigned int num_samples);

/*	Function Name:	gspPadsGlobal_AutoCamera()
 *
 *	Input:			Beacon identifier
 *					Measurements indicating distance and direction to beacon
 *
 *	Purpose:		If the calling beacon is the remote beacon, we'll update our remote information
 *
 */
void gspPadsGlobal_AutoCamera(unsigned int beacon, beacon_measurement_matrix measurements);

/*	Function Name:	gspSetTarget_AutoCamera()
 *
 *
 *	Purpose:		
 *
 */
void gspSetTarget_AutoCamera(unsigned int test_number, unsigned int maneuver_time, 
	float *ctrlStateTarget, unsigned int *maneuver_timeout);

void gspInitProgram_AutoCamera();


/***********************************/
/*   End user-modifiable section   */
/***********************************/

#ifdef __cplusplus
}
#endif

#endif
