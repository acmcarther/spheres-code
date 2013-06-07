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

#include "comm.h"
#include "commands.h"
#include "control.h"
#include "gspFluidSlosh.h"
#include "gsp_task.h"
#include "pads.h"
#include "prop.h"
#include "spheres_constants.h"
#include "spheres_physical_parameters.h"
#include "spheres_types.h"
#include "std_includes.h"
#include "system.h"
#include "util_memory.h"

/*----------------------------------------------------------------------------*/
/*                     Modify as desired below this point.                    */
/*----------------------------------------------------------------------------*/

#include <string.h>

#include "pads.h"
#include "pads_internal.h"
#include "est_StateProp.h"
#include "pads_convert.h"

#include "ctrl_attitude.h"
#include "ctrl_position.h"

#include "ctrl_mix.h"
#include "find_state_error.h"

#include "math_matrix.h"

#include "gsutil_PadsIMU.h"

// For debuging that includes thruster on/off times
#include "gsutil_thr_times.h"

void gspInitTest_BeaconFollow(unsigned int test_number)
{
	extern state_vector initState;

	// Set the background telemetry pointer to the default pads state vector
	commBackgroundPointerDefault();

	// Turn on background telemetry by default
	commBackgroundTelemetryPeriodSet(200);

	// Reset PID controllers
	ctrlPositionPIDinit();
	ctrlAttitudePIDinit();

	// Turn off thruster integration for local propagator only ( still on for global 
	//		estimator )
	initIntThrustCommand(0);

	// Disable IR thruster bypass
	propSetThrusterBypassIR(FALSE);

	// Set the control period
	ctrlPeriodSet(CONTROL_PERIOD);

	// Initialize buffers
	gsutilPadsImuInit(INT_ACCEL_X|INT_ACCEL_Y|INT_ACCEL_Z|INT_GYRO_X|INT_GYRO_Y|
		INT_GYRO_Z, padsInertialSamplePeriodGet(), padsInertialProcessPeriodGet());

	fDataXferDone = 0;
	trans_hap = 1;
}