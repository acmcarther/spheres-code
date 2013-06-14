#ifndef __SPHERES_GSP_H__
#define __SPHERES_GSP_H__

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------------------------------------------------*/
/*                        Begin user-modifiable section                       */
/*----------------------------------------------------------------------------*/

// suggested defines
#define SPHERE_ID       SPHERE1
#define NUM_SPHERES     1
#define NUM_BEACONS     5

#define IMU_BUFFER_LENGTH	50
#define MAX_BEACON_SAMPLES		(4*360*4)	// 4 samples/second for 6 minutes for 4 beacons

/*----------------------------------------------------------------------------*/
/*                         End user-modifiable section                        */
/*----------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif
