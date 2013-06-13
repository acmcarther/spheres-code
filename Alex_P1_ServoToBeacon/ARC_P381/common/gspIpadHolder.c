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
void gspTaskRun_IpadHolder(unsigned int gsp_task_trigger, unsigned int extra_data){}

void gspInitTest_IpadHolder(unsigned int test_number){}

void gspControl_IpadHolder(unsigned int test_number, unsigned int test_time,
	unsigned int maneuver_number, unsigned int maneuver_time){}

void gspPadsInertial_IpadHolder(IMU_sample *accel, IMU_sample *gyro, 
	unsigned int num_samples){}

void gspPadsGlobal_IpadHolder(unsigned int beacon, beacon_measurement_matrix measurements){}

void gspSetTarget_IpadHolder(unsigned int test_number, unsigned int maneuver_time, 
	float *ctrlStateTarget, unsigned int *maneuver_timeout){}

void gspInitProgram_IpadHolder(){}
