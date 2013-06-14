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
#include "pads_convert.h"

#include "ctrl_attitude.h"
#include "ctrl_position.h"

#include "ctrl_mix.h"
#include "find_state_error.h"

#include "math_matrix.h"

#include "gspCustom_glideslope.h"

// Estimator files
#include "est_USRangeBearingPvarEKF.h"
#include "est_StateProp.h"
#include "est_IMUPssEKF.h"

// Core Defines
#define MAX_MANEUVERS 		21		// Is this arbitrary?
#define MANEUVER_TIME_OUT	6000	// Time in ms (I think)

// Basic maneuvers
#define MANEUVER_IDLE	 	0		// Don't do anything at all
#define MANEUVER_HALT	 	1		// Kill any vel or rot

// Complex maneuvers
#define MANEUVER_KALMAN 	2		// Calibrate kalman for single beacon
#define MANEUVER_CHASE		3		// Attempt to move to designated target
#define MANEUVER_HOLD_POS	4		// Continuously kill any vel or rot


// Control variable definitions
static state_vector ctrlState = {0};
static unsigned int targetBeaconNumber = 1;
static float bearingToBeacon[10] = {0};
static float xyzPrimary[BEARING_STATE_LENGTH] = {0}, 
	xyzRefPrimary[BEARING_STATE_LENGTH] = {0}, 
	dxyzPrimary[BEARING_STATE_LENGTH] = {0};
static beacon_measurement_matrix saved_measurements;
static unsigned int fInitFlag = TRUE;
static int fGyroOnly = FALSE, fGl_docking_done = FALSE;

const float sphere_radius = 0.10f;
unsigned int maneuver_nums[MAX_MANEUVERS];
unsigned int maneuver_num_index;

extern const float Kp_attitude_selectObe[21], 
	Kd_attitude_selectObe[21], 
	Kp_position_selectObe[21], 
	Kd_position_selectObe[21], 
	RHO_DOT_0_selectObe[21], 
	RHO_DOT_T_selectObe[21];


/* Utility Functions */

void gspSetTarget_BeaconFollow(unsigned int test_number, unsigned int maneuver_time, 
	float *ctrlStateTarget, unsigned int *maneuver_timeout)
{
	// Declare function variables
	const float range_berth = /*0.10f*/ 1.0f;

	// TODO: Set the target 1-2m away from the target beacon (modify this for that)
	// TODO TODO TODO: This seems to target y only, bad bad, needs to be axis indep.
	memcpy(ctrlStateTarget, xyzRefPrimary, sizeof(float)*3);
	ctrlStateTarget[POS_Y] = ctrlStateTarget[POS_Y] + range_berth + sphere_radius;
	ctrlStateTarget[QUAT_3] = 0.7071068f;
	ctrlStateTarget[QUAT_4] = 0.7071068f;	
}

/* Required functions */

void gspInitTest_BeaconFollow(unsigned int test_number)
{
	// Tell background comm to send this state vector
	commBackgroundPointerSet(&ctrlState);

	// TODO: Turn on background telemetry by default (customize the value)
	commBackgroundTelemetryPeriodSet(200);

	// Set Spheres Beacon
	padsBeaconNumberSet(0);

	// Initialize the beacon estimator
	initStateBearingEKF(targetBeaconNumber);

	// TODO: Initialize glideslope algorithm (custom build a glidescope for this)
	ctrl_initGlideslope();

	// Reset state vectors
	memset(ctrlState,0,sizeof(state_vector));
	ctrlState[QUAT_4] = 1.0f;
	memset(bearingToBeacon,0,sizeof(bearingToBeacon));
	bearingToBeacon[6] = 1.0f;
	fInitFlag = TRUE;
	
	// set the control period
	ctrlPeriodSet(500);

	// Set the maneuver list (could be variable later)
	maneuver_nums[0] 	= MANEUVER_KALMAN;		// Calibration
	maneuver_nums[1] 	= MANEUVER_CHASE;		// Chase test
	maneuver_num_index 	= 0;

}

void gspTaskRun_BeaconFollow(unsigned int gsp_task_trigger, unsigned int extra_data)
{
	// Declare function variables 
	int i;
	const float range_gyro_only = 0.08f; 

	// TODO: Does ctrlState actually have any novel information? Isnt it extra_data

	// Determine if calling function has inertial task	
	if (gsp_task_trigger == PADS_INERTIAL_TRIG)
	{
		// Use new inertial data to update estimated position
		statePropPss(getIMUDATATime(), ctrlState);

		// Use new inertial data to update estimated attitude
		stateGyroUpdatePssEKF(ctrlState);
	}
	
	// Determine if calling function has global task and we're not set to ignore it
	if ((gsp_task_trigger == PADS_GLOBAL_BEACON_TRIG) && !fGyroOnly)
	{

		// Determine if the beacon data is relevant to our target beacon
		if (extra_data == targetBeaconNumber)
		{

			// TODO: Find the XYZ to the target beacon (Cohvert to straight bearing/range)
			xyzUSRangeBearingPvarEKF(extra_data, xyzPrimary);

			// Convert that XYZ to a range and bearing for the controller
			xyz2rqConv(xyzPrimary, bearingToBeacon, XPOSFACE);

			// Loop through the quaternions to calculate the delta
			for (i=0;i<3;i++)
			{
				dxyzPrimary[i] = xyzRefPrimary[i] - xyzPrimary[i];
				dxyzPrimary[i+3] = -xyzPrimary[i+3];
			}
			
			// TODO: Disable global if we're taking too long, are too close, or is not applicable
			if ((ctrlTestTimeGet() > 5000) && ((bearingToBeacon[1] - sphere_radius) < range_gyro_only)) // Only when docking
				fGyroOnly = TRUE;
		}
	}
}	

void gspControl_BeaconFollow(unsigned int test_time, unsigned int maneuver_number, unsigned int maneuver_time)
{
	// TODO: Declare function variables (remove extra variables)
	int init_delay = 10000, 
		min_pulse = 10;
	float ctrlControl[6] = {0}, 
		duty_cycle = 40.0f;
	state_vector ctrlStateError = {0}, 
		ctrlStateTarget = {0};
	//dbg_ushort_packet DebugVecUShort;
	//dbg_float_packet DebugVecFloat;
	prop_time firing_times = {0};
	static float RHO_DOT_0 = 0.0f, 
		RHO_DOT_T = 0.0f, 
		Kp_attitude = 0.0f, 
		Kd_attitude = 0.0f, 
		Kp_position = 0.0f, 
		Kd_position = 0.0f;
	unsigned int maneuver_timeout = 0;

	memset(ctrlControl, 0, sizeof(ctrlControl));
	memset(&firing_times, 0, sizeof(prop_time));
	memset(ctrlStateError, 0, sizeof(state_vector));
	memset(ctrlStateTarget, 0, sizeof(state_vector));

	// Initialize state variables
	if(fInitFlag)
	{
		// TODO: Define maneuvers
		maneuver_num_index = 0;

		// TODO: Set gains (I need to customize these baselines)
		Kp_attitude = 0.0036f;	//Kp_attitude_selectObe[index];
		Kd_attitude = 0.0135f;	//Kd_attitude_selectObe[index];
		Kp_position = 0.172f;	//Kp_position_selectObe[index];
		Kd_position = 1.720f;	//Kd_position_selectObe[index];
				
		// TODO: set glideslope parameters (I need to customize these baselines)
		RHO_DOT_0 = -0.02f;		//RHO_DOT_0_selectObe[index];
		RHO_DOT_T = -0.0005f;	//RHO_DOT_T_selectObe[index];

		// Inititalize inertial sensors
		initStateProp();
		initStateGyroUpdatePssEKF();
		padsInertialPeriodSet(1, padsInertialBufferCapacity());

		// TODO: Initialize data download (Ill do this later)

		// Disable initializer flag
		fInitFlag = FALSE;

	}

	// TODO: Disable global estimator (Can this be done in the test init?)
	padsGlobalPeriodSet(SYS_FOREVER);

	// Fetch position states
	memcpy(ctrlState, dxyzPrimary, sizeof(float)*6);

	// TODO: Set our target state (I need to make a custom version of this)
	gspSetTarget_BeaconFollow(0, maneuver_time, 
		ctrlStateTarget, &maneuver_timeout);

	// Find our difference in state between current and target
	findStateError(ctrlStateError, ctrlState, ctrlStateTarget);

	// Run through our maneuver
	switch(maneuver_number)
	{

		// TODO TODO TODO:: What does this even do? I have no idea
		// Initialize the kalman filter and the reference beacon location
		case MANEUVER_KALMAN:

			// TEMP: Reinitialize estimator after 2s to prevent potential estimator crash
			if ((maneuver_time >= 1800) && (maneuver_time <= 2200))
				initStateBearingEKF(targetBeaconNumber);

			// Controller file
			ctrlAttitudeNLPDwie(0.0f, Kd_attitude, 0.0f, Kd_attitude, 0.0f, 
				Kd_attitude, ctrlStateError, ctrlControl);
				
			// Mixer file
			ctrlMixWLoc(&firing_times, ctrlControl, ctrlState, min_pulse, 
				duty_cycle, FORCE_FRAME_BODY);
				
			// Actuates the thrusters
			propSetThrusterTimes(&firing_times);
			
			// Maneuver terminating conditions
			if (maneuver_time >= init_delay)
			{
				// Reset attitude estimator
				memset(&ctrlState[QUAT_1],0,sizeof(float)*7);
				ctrlState[QUAT_4] = 1;
				initStateProp();
				initStateGyroUpdatePssEKF();

				// Record reference beacon location
				memcpy(xyzRefPrimary, xyzPrimary, sizeof(float)*3);

				maneuver_num_index++;
			}
			break;

		// Point sphere at beacon and maintain 1-2 m distance
		case MANEUVER_CHASE:

			// Set up attitude PD 
			ctrlAttitudeNLPDwie(Kp_attitude, Kd_attitude, Kp_attitude, 
				Kd_attitude, Kp_attitude, Kd_attitude, ctrlStateError, ctrlControl);

			// TODO: Set up position PD (Why is this 'if' statement here?)
			if (test_time % 1000 < 100)
				ctrlPositionPDgains(Kp_position, Kd_position, 0.0f, 0.0f, 
					Kp_position, Kd_position, ctrlStateError, ctrlControl);

			// TODO: Run custom glidescope function (I need to build a custom version of this)
			ctrl_glideslope(maneuver_time, ctrlStateError, ctrlControl, 100, 
				&fGl_docking_done, RHO_DOT_0, RHO_DOT_T, 1, SET_1D); 

			// Thruster Mixing
			ctrlMixWLoc(&firing_times, ctrlControl, ctrlState, min_pulse, 
				duty_cycle, FORCE_FRAME_BODY);

			// Actuate the thrusters
			propSetThrusterTimes(&firing_times);

			// TODO: If maneuver is over move to post-test stage (criteria?)
			if(maneuver_time >= 20000) 
			{
				maneuver_num_index++;
			}

			break;

		default:
			// TODO: Do something befitting of the end of the test

			// Kill all velocity
			break;
		// TODO: Post test data analysis 

}
}

void gspPadsInertial_BeaconFollow(IMU_sample *accel,IMU_sample *gyro, unsigned int num_stored_samples)
{
	// Update our current state information with information from inertial sensor
	setIMUPssEKF_Data(accel, gyro, num_stored_samples);
}

void gspPadsGlobal_BeaconFollow(unsigned int beacon, beacon_measurement_matrix measurements)
{
	// Determine if the beacon from this measurement is the one we care about
	if (beacon == targetBeaconNumber)
	{
		// Store this beacon data for later
		memcpy(saved_measurements, measurements, sizeof(beacon_measurement_matrix));

		// Pass this data to the estimator
		setUSBeacon_Data(beacon, measurements);
	}
}

/* Unimplemented functions */

void gspInitProgram_BeaconFollow(){}
