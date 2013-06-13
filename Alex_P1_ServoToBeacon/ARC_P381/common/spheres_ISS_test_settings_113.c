/* spheres_ISS_test_settings_113.c
 *
 * File containing all the arrays used to set the parameters of the tests
 *
 */

#include "gsp.h"

// attitude gyro only (reconfig)

// general test settings
const unsigned int init_selectConfig[] = {1,3,1,1,3,3,1};

// controller gains
const float Kp_attitude_selectConfig[]={0.0036f,0.0036f,0.0020f,0.0056f,0.0020f,0.0056f,0.0036f};
const float Kd_attitude_selectConfig[]={0.0135f,0.0135f,0.0101f,0.0169f,0.0101f,0.0169f,0.0135f};

// controller gains
const float Kp_attitude_selectObe[]={0.0036f,0.0036f,0.0036f,0.0036f,0.0036f,0.0036f,0.0036f,0.0036f,0.0036f,0.0036f,0.0036f,0.0036f,0.0036f,0.0020f,0.0056f,0.0020f,0.0056f,0.0020f,0.0056f,0.0020f,0.0056f};
const float Kd_attitude_selectObe[]={0.0135f,0.0135f,0.0135f,0.0135f,0.0135f,0.0135f,0.0135f,0.0135f,0.0135f,0.0135f,0.0135f,0.0135f,0.0135f,0.0101f,0.0169f,0.0101f,0.0169f,0.0101f,0.0169f,0.0101f,0.0169f};
const float Kp_position_selectObe[]={0.172f,0.172f,0.172f,0.172f,0.172f,0.172f,0.172f,0.172f,0.172f,0.172f,0.172f,0.172f,0.172f,0.097f,0.269f,0.097f,0.269f,0.097f,0.269f,0.097f,0.269f};
const float Kd_position_selectObe[]={1.720f,1.720f,1.720f,1.720f,1.720f,1.720f,1.720f,1.720f,1.720f,1.720f,1.720f,1.720f,1.720f,1.290f,2.150f,1.290f,2.150f,1.290f,2.150f,1.290f,2.150f};

// glideslope parameters
const float RHO_DOT_0_selectObe[]={-0.02f,-0.02f,-0.02f,-0.02f,-0.02f,-0.02f,-0.02f,-0.02f,-0.02f,-0.02f,-0.02f,-0.02f,-0.02f,-0.02f,-0.02f,-0.02f,-0.02f,-0.02f,-0.02f,-0.02f,-0.02f};
const float RHO_DOT_T_selectObe[]={-0.005f,-0.005f,-0.005f,-0.005f,-0.005f,-0.005f,-0.005f,-0.005f,-0.005f,-0.005f,-0.005f,-0.005f,-0.005f,-0.005f,-0.005f,-0.005f,-0.005f,-0.005f,-0.005f,-0.005f,-0.005f};



