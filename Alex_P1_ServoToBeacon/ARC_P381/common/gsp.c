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

// pick ONE
#define LAB_VERSION
//#define ISS_VERSION

#ifdef ISS_VERSION
#define DEFAULT_X 0.0
#define THE_PROGRAM_NUMBER 999 // Make a proper number for this
#else //LAB_VERSION
#define DEFAULT_X -0.67
#define THE_PROGRAM_NUMBER 223 // Make a proper number for this
#endif

#define QX 0
#define QY 1
#define QZ 2
#define QW 3

// translation margin could be 0.05 in orbit, but with the sphere sideways, want it bigger
// especially for testing now
// was 0.09
#define TRANSLATION_MARGIN 0.10
#ifdef ISS_VERSION
#define X_MARGIN TRANSLATION_MARGIN
#else // lab version
#define X_MARGIN TRANSLATION_MARGIN
#endif

#define VELOCITY_MARGIN 0.05
// 0.35 rad ~ 20 degrees
#define QUAT_AXIS_MARGIN 0.35
//#define QUAT_ANGLE_MARGIN 0.99
#define RATE_MARGIN 0.1
#define EPSILON 0.01
#define TIMED_OUT 2

#ifndef _TMS320C6X
	#define DEBUG(arg)  mexprintf arg
#else
	#define DEBUG(arg)
#endif
// then write: DEBUG(("  ", ui));


/*	This is where my own defines start	 */

#define SERVO_TO_BEACON_TEST 1
#define FLOATING_IPAD_HOLDER_TEST 2
#define AUTONOMOUS_CAMERA_TEST 3

/*	This is where my own defines stop	*/


int maneuver_nums[MAX_MANEUVERS];
int maneuver_num_index;

static int testclass;
state_vector ctrlStateTarget;
state_vector commandStateTarget;
char stopAtEnd;
unsigned char global_target_reached = FALSE;
unsigned char global_sphere_error = FALSE;
unsigned short global_last_cmd = 0;
unsigned char global_add_translate = TRUE;

unsigned char cnt;
unsigned char gpio_out[16];
extern state_vector initState;

// callback function prototype
void gspProcessPhoneCommand(unsigned char channel, unsigned char* buffer, unsigned int len);

void gspIdentitySet()
{
	// set the logical identifier (SPHERE#) for this vehicle
	sysIdentitySet(SPHERE_ID);
}

void gspInitProgram()
{
	// set the unique program identifier (to be assigned by MIT)
	sysProgramIDSet(THE_PROGRAM_NUMBER);

	// set up communications TDMA frames
	commTdmaStandardInit(COMM_CHANNEL_STL, sysIdentityGet(), 
		NUM_SPHERES);
	commTdmaStandardInit(COMM_CHANNEL_STS, sysIdentityGet(), 
		NUM_SPHERES);
	commTdmaStandardInit(COMM_CHANNEL_EXP, sysIdentityGet(), 
		NUM_SPHERES);

	// enable communications channels
	commTdmaEnable(COMM_CHANNEL_STL);
	commTdmaEnable(COMM_CHANNEL_EXP);
   
	// allocate storage space for IMU samples
	padsInertialAllocateBuffers(50);

	// inform system of highest beacon number in use
	padsInitializeFPGA(NUM_BEACONS);

	/* custom program initialization goes below this point */	
	maneuver_num_index = 0;
   	
	*SMT335CP4 = 0x1104;// Talk to the Expansion Board at 250 kbps

	// Determine if we still need any of this stuff	
	expv2_init(); // still needed? not in VERTIGO_ExpV2_Testing
	expv2_uart_cbk_register(1,&gspProcessPhoneCommand);
	expv2_uart_baud_set(1,115200);
}
    	
void gspInitTest(unsigned int test_number)
{
	/* TODO:
	 *	Ideally this function should handle all of the different 
	 *	experiments, therefore we need to set up the test numbers
	 *	
	 *	Test1: Servo to Beacon or "Chase the human"
	 *	Test2: Floating iPad holder
	 *	Test3: Autonomus free flying camera holder
	 *
	 */

	// Set the control query period
	ctrlPeriodSet(1000);

	// TODO: Determine what this does
	memset(gpio_out,0,sizeof(gpio_out));

	// TODO: Find out the analogue of this estimator in the old code
	#if (SPHERE_ID == SPHERE1)
		padsEstimatorInitWaitAndSet(initState, 50, 200, 105,
		PADS_INIT_THRUST_INT_ENABLE,PADS_BEACONS_SET_1TO9); // ISS 
	#else
		padsEstimatorInitWaitAndSet(initState, 50, SYS_FOREVER, SYS_FOREVER,
		PADS_INIT_THRUST_INT_ENABLE,PADS_BEACONS_SET_1TO9); // ISS
	#endif

	// TODO: Determine what this does
	memset(ctrlStateTarget,0,sizeof(state_vector));
	ctrlStateTarget[POS_X] = DEFAULT_X;
	ctrlStateTarget[QUAT_1] = 0;
	ctrlStateTarget[QUAT_2] = 0;
	ctrlStateTarget[QUAT_3] = 0;
	ctrlStateTarget[QUAT_4] = 0;
	memcpy(commandStateTarget, ctrlStateTarget, sizeof(state_vector));
    
	// Initialize individual tests
	switch (test_number)
	{
		case SERVO_TO_BEACON_TEST: 
			break;
		case FLOATING_IPAD_HOLDER_TEST:
			break;
		case AUTONOMOUS_CAMERA_TEST:
			break;
	}
}

void gspInitTask()
{
	// TODO: Initialize some specific subtasks
}

void gspPadsInertial(IMU_sample *accel, IMU_sample *gyro, unsigned int num_samples)
{
	// TODO: Act on the new inertial data based on the current test
	switch (testclass)
	{
		case CHECKOUT:
			break;
		default:
			break;
	}
}

void gspPadsGlobal(unsigned int beacon, beacon_measurement_matrix measurements)
{
	// TODO: Act on the new global data based on the current test
}

void gspTaskRun(unsigned int gsp_task_trigger, unsigned int extra_data)
{
	// TODO: Execute some task
	switch (testclass)
	{
		case CHECKOUT:
			// TODO: find and put back in
			gspTaskRun_Checkout(gsp_task_trigger,extra_data);
			break;
		default:
			break;
	}
}

void send_SOH_packet_to_phone() {
	// TODO: I assume I do not have to change any of this code

	comm_payload_soh my_soh;
	comm_payload_telemetry my_position;
 	
	// get my SOH information
	commBackgroundSOHGet(SPHERE_ID, &my_soh);	
	
	// set the fields I need
	my_soh.unused[0] = global_target_reached;
	my_soh.unused[1] = global_sphere_error;
	my_soh.unused[3] = (global_last_cmd>>8) & 0xFF; // <<-- is this right?
	my_soh.unused[2] = global_last_cmd & 0xFF;
	
	// send it
	expv2_uart_send_w_het_header(EXPv2_CH1_HWID,
		sizeof(comm_payload_soh), (unsigned char *)&my_soh,
		COMM_CMD_SOH);


	commBackgroundPayloadPack(&my_position);
	expv2_uart_send_w_het_header(EXPv2_CH1_HWID, 
		sizeof(comm_payload_telemetry), 
		(unsigned char *)&my_position, COMM_CMD_BACKGROUND);
}

void gspControl(unsigned int test_number, unsigned int test_time, unsigned int maneuver_number, unsigned int maneuver_time)
{	
	// TODO: cut the stuff I dont need out of here
	state_vector ctrlState; // current state vector of the sphere
	state_vector ctrlStateError; // difference btwn ctrlState and ctrlStateTarget
	float ctrlControl[6];
	prop_time firing_times;
	const int min_pulse = 10;
	extern const float KPattitudePD, KDattitudePD, KPpositionPD, KDpositionPD;
	dbg_float_packet dbg_target;
	dbg_short_packet dbg_error;
	extern state_vector initState;
//	comm_payload_soh my_soh;
 
	//Clear all uninitialized vectors
	memset(ctrlControl,0,sizeof(float)*6);
	memset(ctrlStateError,0,sizeof(state_vector));

	memset(dbg_target,0,sizeof(dbg_float_packet));
	memset(dbg_error,0,sizeof(dbg_short_packet));
    
//	memset(my_soh,0,sizeof(comm_payload_soh));
   
	padsStateGet(ctrlState);
    
	if(testclass == CHECKOUT) 
	{
		gspControl_Checkout(test_number, test_time, maneuver_number, maneuver_time);
		return;
	}
	
	if (maneuver_number == CONVERGE_MODE) //Estimator initialization
	{
		if (test_time >= ESTIMATOR_TIME)
		{
			maneuver_num_index++;
			ctrlManeuverNumSet(maneuver_nums[maneuver_num_index]);
			memcpy(ctrlStateTarget,ctrlState,sizeof(state_vector));
			ctrlStateTarget[VEL_X]=0.0f;
			ctrlStateTarget[VEL_Y]=0.0f;
			ctrlStateTarget[VEL_Z]=0.0f;
			ctrlStateTarget[RATE_X]=0.0f;
			ctrlStateTarget[RATE_Y]=0.0f;
			ctrlStateTarget[RATE_Z]=0.0f;
		}
		
	}
	else if (maneuver_number == DRIFT_MODE) 
	{
		// do nothing - just drift!
	}
	else if (maneuver_number == WAYPOINT_MODE)
	{
		//Disable estimator during closed loop firing
		padsGlobalPeriodSet(SYS_FOREVER);

		//find error
		findStateError(ctrlStateError,ctrlState,ctrlStateTarget); 
        
		//call controllers
		ctrlPositionPDgains(KPpositionPD, KDpositionPD, KPpositionPD, 
			KDpositionPD, KPpositionPD, KDpositionPD, 
			ctrlStateError, ctrlControl);
		ctrlAttitudeNLPDwie(KPattitudePD,KDattitudePD,KPattitudePD,
			KDattitudePD,KPattitudePD,KDattitudePD, 
			ctrlStateError,ctrlControl);

		//mix forces/torques into thruster commands
		ctrlMixWLoc(&firing_times, ctrlControl, ctrlState, 
			min_pulse, 20.0f, FORCE_FRAME_INERTIAL);

		//Set firing times
		propSetThrusterTimes(&firing_times);

		#if (SPHERE_ID == SPHERE1)
			padsGlobalPeriodSetAndWait(200,205);
		#endif

		// termination conditions
		// (maneuver_time > ESTIMATOR_TIME)
		if ((atPositionRotation(ctrlStateError)))
		{
			// if rotation test, just need x correct
			// if maneuver 20 or not rotation test, need x and 
			//	x_dot correct
			if((!stopAtEnd) || (atZeroVelocity(ctrlStateError)))
			{
				//  we got there
				global_target_reached = TRUE;
			}
		} 
		if(maneuver_time > MANEUVER_TIME_OUT) {
			global_target_reached = TIMED_OUT;
		}	
	}
	
	dbg_target[0] = maneuver_time;
	dbg_target[1] = ctrlStateTarget[POS_X];
	dbg_target[2] = ctrlStateTarget[POS_Y];	
	dbg_target[3] = ctrlStateTarget[POS_Z];
	dbg_target[4] = ctrlStateTarget[QUAT_1];
	dbg_target[5] = ctrlStateTarget[QUAT_2];
	dbg_target[6] = ctrlStateTarget[QUAT_3];
	dbg_target[7] = ctrlStateTarget[QUAT_4];	
	
	dbg_error[0] = maneuver_time/1000;
	dbg_error[1] = ctrlStateError[POS_X]*1000;
	dbg_error[2] = ctrlStateError[POS_Y]*1000;
	dbg_error[3] = ctrlStateError[POS_Z]*1000;
	dbg_error[4] = 0;
	dbg_error[5] = ctrlStateError[QUAT_1]*1000;
	dbg_error[6] = ctrlStateError[QUAT_2]*1000;
	dbg_error[7] = ctrlStateError[QUAT_3]*1000;
	dbg_error[8] = ctrlStateError[QUAT_4]*1000;
	dbg_error[9] = fabs(getQuaternionMagnitude(ctrlStateError[QUAT_4]))
		*1000.0;
	dbg_error[10] = QUAT_AXIS_MARGIN*1000.0;
	dbg_error[11] = global_target_reached;
		
	send_SOH_packet_to_phone();

	commSendRFMPacket(COMM_CHANNEL_STL, GROUND, COMM_CMD_DBG_FLOAT, 
		(unsigned char *) dbg_target, 0);
	commSendRFMPacket(COMM_CHANNEL_STL, GROUND, COMM_CMD_DBG_SHORT, 
		(unsigned char *) dbg_error, 0);
	
}

int checksumChecks(unsigned char* buffer, unsigned int len) {
	int i;
	unsigned short bigcheck=0;
	
	dbg_short_packet dbg_error;
	phone_cmd* cmd = (phone_cmd*) buffer;
	
	if(len != sizeof(phone_cmd))
		return FALSE;
		
	for(i=8; i<sizeof(phone_cmd); i++)
		bigcheck += buffer[i];
	
	dbg_error[0] = bigcheck;
	dbg_error[1] = cmd->hdr.chk;
	
	commSendRFMPacket(COMM_CHANNEL_STL, GROUND, COMM_CMD_DBG_SHORT_UNSIGNED,
		(unsigned char *) dbg_error, 0);

	if(bigcheck == cmd->hdr.chk)
		return TRUE;
		
	return FALSE;
}

void gspProcessPhoneCommand(unsigned char channel, unsigned char* buffer, unsigned int len)
{
	// TODO: Find out if this is this necessary
	state_vector ctrlState; // current state vector of the sphere
	phone_cmd* cmd = (phone_cmd*)buffer;
	float x2, y2, z2, w2;
	float target_quat[4];
	dbg_float_packet dbg_target;
	
//    dbg_short_packet dbg_error;
    //expv2_uart_send_w_het_header(EXPv2_CH1_HWID, len, buffer, COMM_CMD_SOH);
  
	dbg_target[0] = 555;
	dbg_target[1] = 555;
	dbg_target[2] = 555;
	dbg_target[3] = cmd->cmd;
	dbg_target[4] = cmd->cmd;
	dbg_target[5] = cmd->cmd;
	dbg_target[6] = cmd->cmd;
	dbg_target[7] = 555;			
//	commSendRFMPacket(COMM_CHANNEL_STL, GROUND, COMM_CMD_DBG_SHORT, (unsigned char *) dbg_error, 0);
	commSendRFMPacket(COMM_CHANNEL_STL, GROUND, COMM_CMD_DBG_FLOAT, 
		(unsigned char *) dbg_target, 0);

	// check the checksum
	if(!checksumChecks(buffer, len)) {
		return;
	}

	if(cmd->seq_num <= global_last_cmd)
		return; // I've seen this command before

	// acknowledge that I received this command correctly
	global_last_cmd = cmd->seq_num;

	if(cmd->cmd == JUST_DRIFT)
	{
		//ctrlManeuverNumSet(DRIFT_MODE);
		send_SOH_packet_to_phone();
		
		ctrlTestTerminate(TEST_RESULT_NORMAL);
		return;
	}
		
	// find out where we (think we) are
	padsStateGet(ctrlState);
	
	switch(cmd->cmd)
	{
		case GO_TO_XYZ:
			// aim for commanded position
			commandStateTarget[POS_X] = cmd->x * INV_POSITION_SCALE;
			commandStateTarget[POS_Y] = cmd->y * INV_POSITION_SCALE;
			commandStateTarget[POS_Z] = cmd->z * INV_POSITION_SCALE;
				
			ctrlManeuverNumSet(WAYPOINT_MODE);
			stopAtEnd = cmd->stop_at_end;
			global_target_reached = FALSE;
			memcpy( ctrlStateTarget, commandStateTarget, sizeof(state_vector));
			break;
				
		case GO_TO_QUAT:
			// Updates quat that goes in the commandStateVector. Meaning ... 
			// we are going to over-ride all past relative/manual movements.

			// aim for commanded orientation
			commandStateTarget[QUAT_1] = cmd->qx * INV_QUATERNION_SCALE;
			commandStateTarget[QUAT_2] = cmd->qy * INV_QUATERNION_SCALE;
			commandStateTarget[QUAT_3] = cmd->qz * INV_QUATERNION_SCALE;
			commandStateTarget[QUAT_4] = cmd->qw * INV_QUATERNION_SCALE;
			ctrlManeuverNumSet(WAYPOINT_MODE);
			stopAtEnd = cmd->stop_at_end;
			global_target_reached = FALSE;
			memcpy( ctrlStateTarget, commandStateTarget, sizeof(state_vector));
			break;

		case RELATIVE_XYZ:
			// This operates relative to actual sphere location.
		
			// aim for current position plus command
			ctrlStateTarget[POS_X] = ctrlState[POS_X] + cmd->x * INV_POSITION_SCALE;
			ctrlStateTarget[POS_Y] = ctrlState[POS_Y] + cmd->y * INV_POSITION_SCALE;
			ctrlStateTarget[POS_Z] = ctrlState[POS_Z] + cmd->z * INV_POSITION_SCALE;
				
			ctrlManeuverNumSet(WAYPOINT_MODE);
			stopAtEnd = cmd->stop_at_end;
			global_target_reached = FALSE;
			break;
				
		case RELATIVE_QUAT:
			// This operates relative to actual sphere location.
		
			// aim for current position

			// aim for current orientation plus command ... wait ...
			x2 = cmd->qx * INV_QUATERNION_SCALE;
			y2 = cmd->qy * INV_QUATERNION_SCALE;
			z2 = cmd->qz * INV_QUATERNION_SCALE;
			w2 = cmd->qw * INV_QUATERNION_SCALE;
			
			gspRotateByQuaternion(x2, y2, z2, w2,						   
						   ctrlState[QUAT_1], ctrlState[QUAT_2], ctrlState[QUAT_3], ctrlState[QUAT_4], 
						   target_quat);

			ctrlStateTarget[QUAT_1] = target_quat[0];
			ctrlStateTarget[QUAT_2] = target_quat[1];
			ctrlStateTarget[QUAT_3] = target_quat[2];
			ctrlStateTarget[QUAT_4] = target_quat[3];

			ctrlManeuverNumSet(WAYPOINT_MODE);
			stopAtEnd = cmd->stop_at_end;
			global_target_reached = FALSE;
			break;
			
		case HOLD_POSITION:
			// This tells us to hold actual sphere location
		
			// We DO need to read the current state for this
			ctrlStateTarget[POS_X] = ctrlState[POS_X];
			ctrlStateTarget[POS_Y] = ctrlState[POS_Y];
			ctrlStateTarget[POS_Z] = ctrlState[POS_Z];
				
			ctrlStateTarget[QUAT_1] = ctrlState[QUAT_1];
			ctrlStateTarget[QUAT_2] = ctrlState[QUAT_2];
			ctrlStateTarget[QUAT_3] = ctrlState[QUAT_3];
			ctrlStateTarget[QUAT_4] = ctrlState[QUAT_4];
			ctrlManeuverNumSet(WAYPOINT_MODE);
			stopAtEnd = cmd->stop_at_end;
			global_target_reached = FALSE;
			break;
		}

 }

// necessary to compile
void gspProcessRXData() {}
