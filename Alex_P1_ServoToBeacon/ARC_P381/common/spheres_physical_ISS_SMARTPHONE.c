/*
 * spheres_physical_ISS.c
 *
 * File written to include the vehicles properties 
 *
 * This file is for the ISS environment
 *						
 */


#include "spheres_types.h"


// Recommended EKF initialization vector
state_vector initState = {0.0f,0.0f,0.0f,  0.0f,0.0f,0.0f,  0.0f,0.0f,0.0f,1.0f,  0.0f,0.0f,0.0f};

const unsigned char PHYS_PROP = 2;

const unsigned int DOCK_MECHANISM_PRESENT	= 0;

// 4.3g sphere + 0.174g smartphone + 0.142g batteries
const float VEHICLE_MASS = 4.616f;      			// [kg] Sphere mass with about 1/2 fuel capacity
const float VEHICLE_ROT_ACCEL = 0.48f;			// [rad/s/s] Vehicle Rotational acceleration
const float VEHICLE_LIN_ACCEL = 0.026f;			// [m/s] Vehicle Linear acceleration  
const float VEHICLE_THRUST_FORCE = 0.112f; 		// [N] = DEFAULT_THRUSTER_FORCE in ISS
const float VEHICLE_MOMENT_ARM = 0.0965f;       // [m] Thruster Moment Arm 
const float DEFAULT_THRUSTER_FORCE = 0.112f;    // [N] baseline thruster force (S/N 2)

const float VEHICLE_THRUST_FORCE_MULTIPLIER = 1.0f;	// maps efficient thruster force to real thruster force in ctrl_mixer_1

// Suggested gains

// PD controller
const float KPattitudePD = 0.0036f; // Proportional gain for attitude control wn=0.400 rad/s
const float KDattitudePD = 0.0135f; // Derivative gain for attitude control wn=0.400 rad/s with 25% damping reduction
const float KPpositionPD = 0.172f; // Proportional gain for position control wn=0.200 rad/s
const float KDpositionPD = 1.720f; // Derivative gain for position control wn=0.200 rad/s

// PID controller (the integral term is set for a time constant of approximately 20 sec)
const float KPattitudePID = 0.0045f; // Proportional gain for attitude control wn=0.400 rad/s
const float KIattitudePID = 0.00018f; // Derivative gain for attitude control wn=0.400 rad/s
const float KDattitudePID = 0.0143f; // Derivative gain for attitude control wn=0.400 rad/s with 25% damping reduction
const float KPpositionPID = 0.258f; // Proportional gain for position control wn=0.200 rad/s
const float KIpositionPID = 0.0086f; // Derivative gain for position control wn=0.200 rad/s
const float KDpositionPID = 1.935f; // Derivative gain for position control wn=0.200 rad/s
