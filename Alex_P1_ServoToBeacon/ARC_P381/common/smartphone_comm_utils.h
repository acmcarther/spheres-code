/*
 *  smartphone_comm_utils.h
 *  
 *
 *  Created by Wheeler, DW  (ARC-TI)[Stinger Ghaffarian Technologies Inc. (SGT Inc.)] on 3/14/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef SMARTPHONE_COMM_UTILS
#define SMARTPHONE_COMM_UTILS

#define GO_TO_XYZ		1
#define GO_TO_QUAT		2
#define RELATIVE_XYZ	3
#define RELATIVE_QUAT	4
#define HOLD_POSITION	5
#define JUST_DRIFT		6

#define SERIAL_DEVICE_1	1
#define SERIAL_DEVICE_2	2

typedef struct{
	unsigned char preamble[4];
	short chk;
	unsigned char cmd;
	unsigned char len;
} het_header;

typedef struct{
	het_header hdr;
	short x; // target X coordinate in m [-0.45,0.45]
	short y; // target Y coordinate in m [-0.80,0.80]
	short z; // target Z coordinate in m [-0.65,0.65]
	short qx; // target Qx
	short qy; // target Qy
	short qz; // target Qz
	short qw; // target Qw
	unsigned short seq_num; // sequence number of this command
	unsigned char stop_at_end; // include zero velocity as termination criteria
	unsigned char cmd; // identifies which command

} phone_cmd;

void expv2_uart_send_w_het_header(unsigned char channel, unsigned char len, unsigned char *data, unsigned char cmd);	


#endif
