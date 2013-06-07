#ifndef __SPHERES_GSPBEACONFOLLOW__
#define __SPHERES_GSPBEACONFOLLOW__

#ifdef __cplusplus
extern "C" {
#endif


/***********************************/
/*  Begin user-modifiable section  */
/***********************************/

#include "pads.h"

void gspInitTest_BeaconFollow(unsigned int test_number);
void gspControl_BeaconFollow(unsigned int test_number, unsigned int test_time,
	unsigned int maneuver_number, unsigned int maneuver_time);
void gspPadsInertial_BeaconFollow(IMU_sample *accel, IMU_sample *gyro, 
	unsigned int num_samples);
void gspPadsGlobal_BeaconFollow(unsigned int beacon, beacon_measurement_matrix measurements);

/***********************************/
/*   End user-modifiable section   */
/***********************************/

#ifdef __cplusplus
}
#endif

#endif
