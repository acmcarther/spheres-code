 /* ctrl_glideslope.c
 *
 * Arthur Richards, Simon Nolet
 *
 * 2001-02-20 - created file
 * 2002-07-03 - made 2D
 * 2002-07-25 - mod by Arthur to make relative docking
 *              feed it state [pos; vel] in [xm1 .. xm6]
 *              it will work to make this relative state zero
 * 2003-05-30 - adapted to flight hardware
 * 2003-06-05 - adapted to lab tests (included carriage mass)
 * 2006-03-02 - adapted to discrete control scheme where pulses
 *				occur at the beginning of a control period
 *			  - now has the option to be unidirectional
 *
 *
 * Note: This function must be called after any attitude controller
 *		 (it cancels the attitude corrections)
 */

/***************************************************************************/

#include "gspCustom_glideslope.h"
#include "control.h"
#include "spheres_constants.h"
#include <string.h>
#include <math.h>


static int N_P=0;  // number of firings allowed for the approach
static unsigned int fInitFlag=0;

static float uRho[3] = {0};
float xm[6];
static float dv[3] = {0};   

// glide slope parameters
static float rho0 = 0.0f;
static float a = 0.0f; 
static float T = 0.0f;

// controller housekeeping states
static float nextPulse = 0.0f;
static int m = 0;
static int p = 0;
static float tManStart = 0.0f;   
static float dv_max = 0.0f;			
static float thruster_force = 0.0f;
	
 
void ctrl_glideslope(unsigned int maneuver_time, float *ctrlStateError, float *ctrlControl, float duty_cycle, int *fGl_docking_done, float RHO_DOT_0, float RHO_DOT_T, int P_MAX, unsigned int fDOF)
{
	float tt;
	float rhoNext = 0.0f;
	float xyzNext = 0.0f;
	int ii;
	unsigned int nextPulseTemp = 0;
	unsigned int actPulseTemp = 0;	

	if (fInitFlag)
	{
		// number of firings
		if (P_MAX>1)
			N_P = (int) floor(fabs(ctrlStateError[POS_Y]*10.0f));
		else
			N_P = (int) floor(2.0f*fabs(ctrlStateError[POS_Y]*10.0f));

		if (N_P < 1)
			N_P = 1; // make sure there is at least 1 firing
			
		thruster_force = (duty_cycle*0.01f) * VEHICLE_THRUST_FORCE; // equivalent force over a full control period
		
		// max delta-v in 1 pulse (1 pulse per control period)
		dv_max = 0.001f * ctrlPeriodGet() * 2 * thruster_force / VEHICLE_MASS; // in m/s
	
		fInitFlag = 0;
	}
	
	// convert to time in seconds
	tt=maneuver_time/1000.0f;
	
	// at start of maneuver, reset controller states
	if (tt <= 1.0) {
		m = 0;
		p = 0;
		nextPulse = 1.2f;
	}
	
	// check if pulse needed
	if (tt >= nextPulse) {
	
		// extract state from arguments
		for (ii=0;ii<6;ii++)
			xm[ii]=ctrlStateError[POS_X+ii];
	
		// if entering with m=0, this is start of new maneuver
		if (m < 1) {
			
			// calculate approach range
			if (fDOF==SET_3D)
				rho0 = sqrt((xm[0]*xm[0]) + (xm[1]*xm[1]) + (xm[2]*xm[2])); // in m
			else
				rho0 = fabs(xm[1]);
				
			// calculate approach direction
			if (fDOF==SET_3D)
			{
				for (ii=0; ii<3; ii++)
					uRho[ii] = xm[ii] / rho0;
			}
			else
			{
				uRho[1] = xm[1] / rho0;
			}
			
			// glideslope
			a = (RHO_DOT_0 - RHO_DOT_T)/rho0; // in 1/s
			
			// transfer time
			T = (log(RHO_DOT_T/RHO_DOT_0))/a; // in s

			// record starting time
			tManStart = tt;
			
		} //end for if (m < 1)
	
		// check if this is continuation of firing
		if (p < 1) {  // (if not continuation of firing)
		
			// check if right number of pulses already done
			if (m < N_P) {
				// calc next rho to aim for
				nextPulseTemp = (unsigned int) ceil(T*(m+1)/N_P);
				actPulseTemp = (unsigned int) ceil(T*m/N_P);
				rhoNext = rho0*exp(a*nextPulseTemp) + (RHO_DOT_T/a)*(exp(a*nextPulseTemp) - 1.0f); // in m
				
				// find control for each axis
				for (ii=0; ii<3; ii++) {
					// calc next target for this axis
					xyzNext = rhoNext*uRho[ii]; // in m
					
					// calc velocity needed to get there
					dv[ii] = -(xyzNext - xm[ii])/(nextPulseTemp-actPulseTemp); // in m/s
					
					// subtract current velocity to get dv
					dv[ii] += xm[ii + 3]; // in m/s
				  
				} //end for for (ii=0; ii<3; ii++)
			} //end for if (m < N_P)
			// else should be at standstill
			else {
				// find control to stand still
				for (ii=0; ii<3; ii++) {
					dv[ii] = xm[ii + 3]; // in m/s
				}
			} //end for else and (m < N_P)
			
			// initially to be done in one pulse
			p = 1;
			
			// spread across pulses to eliminate saturation
			while ((p < P_MAX)&((fabs(dv[0]) > dv_max)|(fabs(dv[1]) > dv_max)|(fabs(dv[2]) > dv_max))) {
				p++;
				dv[0] = dv[0]*(p-1)/p;
				dv[1] = dv[1]*(p-1)/p;
				dv[2] = dv[2]*(p-1)/p;
			}
		
			// update controller state
			m++;
		}
	
		// start working down through pulses
		if (p > 0) {
		
			// next pulse coming up
			p--;
			
			// disable attitude control while firing for delta-V
			//ctrlControl[TORQUE_X] = 0.0;
			//ctrlControl[TORQUE_Y] = 0.0;
			//ctrlControl[TORQUE_Z] = 0.0;
			
			// move on if firing finished
			if (p == 0) {
				nextPulse = tManStart + ceil(m * T / N_P);
				if (m == N_P)
					*fGl_docking_done = 1;
			}
		}
	}
	// else not firing right now
	else {
		// set dv to zero
		memset(dv,0,sizeof(dv));
	}
	
	// write delta-V to output globals, scaled to be force
	if (fDOF==SET_3D)
	{
		ctrlControl[FORCE_X] = dv[0] * 1000 / ctrlPeriodGet() * VEHICLE_MASS; // in N
		ctrlControl[FORCE_Y] = dv[1] * 1000 / ctrlPeriodGet() * VEHICLE_MASS; // in N
		ctrlControl[FORCE_Z] = dv[2] * 1000 / ctrlPeriodGet() * VEHICLE_MASS; // in N
	}
	else
		ctrlControl[FORCE_Y] = dv[1] * 1000 / ctrlPeriodGet() * VEHICLE_MASS; // in N
	
	DebugVecShort[6] = (short) (N_P);	
	DebugVecShort[7] = (short) (m);
	DebugVecShort[8] = (short) (p);
	DebugVecShort[9] = (short) (T * 100.f);
	DebugVecShort[10] = (short) (rho0 * 10000.f);
	DebugVecShort[11] = (short) (tManStart*100.f);
	DebugVecShort[12] = (short) (tt*100.f);
	DebugVecShort[13] = (short) (nextPulse*100.f);
	DebugVecShort[14] = (short) (dv[1]*10000.f);
	DebugVecShort[15] = (short) *fGl_docking_done;
}



void ctrl_initGlideslope()
{
	//Make sure that initializer works properly for ctrlStateTarget
	memset(dv,0,sizeof(dv));
	memset(xm,0,sizeof(xm));
	memset(uRho,0,sizeof(uRho));
	
	N_P=0;
	fInitFlag = 1;
	
	// glide slope parameters
	rho0 = 0.0f;
	a = 0.0f; 
	T = 0.0f;
	
	// controller housekeeping states
	nextPulse = 0.0f;
	m = 0;
	p = 0;
	tManStart = 0.0f;
	dv_max = 0.0f;			
	thruster_force = 0.0f;
}
