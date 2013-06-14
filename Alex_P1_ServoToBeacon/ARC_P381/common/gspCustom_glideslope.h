/*
 * ctrl_glideslope.h
 *
 * Header file for glideslope.c
 *
 * Change Log:
 *
 */

#ifndef __CTRL_GLIDESLOPE__
#define __CTRL_GLIDESLOPE__

#ifdef __cplusplus
extern "C" {
#endif

#include "spheres_types.h"

#define SET_3D 0
#define SET_1D 1

// defined in spheres_physical_*.c
extern const float VEHICLE_THRUST_FORCE;
extern const float VEHICLE_MASS;

//extern dbg_short_packet DebugVecShort;

void ctrl_glideslope(unsigned int maneuver_time, float *ctrlStateError, float *ctrlControl, float duty_cycle, int *fGl_docking_done, float RHO_DOT_0, float RHO_DOT_T, int P_MAX, unsigned int fDOF);
void ctrl_initGlideslope();


#ifdef __cplusplus
}
#endif

#endif

