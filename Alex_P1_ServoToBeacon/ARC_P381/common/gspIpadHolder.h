#ifndef __SPHERES_GSPIPADHOLDER__
#define __SPHERES_GSPIPADHOLDER__

#ifdef __cplusplus
extern "C" {
#endif


/***********************************/
/*  Begin user-modifiable section  */
/***********************************/
#include "spheres_types.h"
#include "pads.h"


/*	Function Name:	gspTaskRun_IpadHolder()
 *
 *	Input:			The event identifier that called this function
 *					Any extra information needed to resolve the task
 *
 *	Purpose:		Handle any relevant tasks for the sphere	
 *
 */
void gspTaskRun_IpadHolder(unsigned int gsp_task_trigger, unsigned int extra_data);

/*	Function Name:	gspInitTest_IpadHolder()
 *
 *	Input:			Number indicating a subtest to execute
 *
 *	Purpose:		Initialize the custom functionality of this test
 *
 */
void gspInitTest_IpadHolder(unsigned int test_number);

/*	Function Name:	gspControl_IpadHolder()
 *
 *	Input:			Test initialization time
 *					Identifier for the maneuver					
 *					Maneuver run time
 *
 *	Purpose:		Execute the control sequences for the sphere	
 *
 */
void gspControl_IpadHolder(unsigned int test_number, unsigned int test_time,
	unsigned int maneuver_number, unsigned int maneuver_time);

/*	Function Name:	gspPadsInertial_IpadHolder()
 *
 *	Input:			Data samples of the internal gyro
 *					Total number of samples
 *
 *	Purpose:		Implement some action based on new inertial data
 *
 */
void gspPadsInertial_IpadHolder(IMU_sample *accel, IMU_sample *gyro, 
	unsigned int num_samples);

/*	Function Name:	gspPadsGlobal_IpadHolder()
 *
 *	Input:			Beacon identifier
 *					Measurements indicating distance and direction to beacon
 *
 *	Purpose:		If the calling beacon is the remote beacon, we'll update our remote information
 *
 */
void gspPadsGlobal_IpadHolder(unsigned int beacon, beacon_measurement_matrix measurements);

/*	Function Name:	gspSetTarget_IpadHolder()
 *
 *
 *	Purpose:		
 *
 */
void gspSetTarget_IpadHolder(unsigned int test_number, unsigned int maneuver_time, 
	float *ctrlStateTarget, unsigned int *maneuver_timeout);

void gspInitProgram_IpadHolder();


/***********************************/
/*   End user-modifiable section   */
/***********************************/

#ifdef __cplusplus
}
#endif

#endif
