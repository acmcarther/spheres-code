#include "exp_v2.h"
#include "smartphone_comm_utils.h"

void expv2_uart_send_w_het_header(unsigned char channel, unsigned char len, unsigned char *data, unsigned char cmd) {
        short big_chk = 0;
        het_header het_hdr;
        int i;
 	               
        for (i=0; i < len; i++)
                big_chk += data[i];
 	
        // make our HET header
        het_hdr.preamble[0] = 0xAA;
        het_hdr.preamble[1] = 0x55;
        het_hdr.preamble[2] = 0xAA;
        het_hdr.preamble[3] = 0x55;             
		het_hdr.chk = big_chk;
		het_hdr.cmd = cmd;
        het_hdr.len = len;
 	       
       expv2_uart_send(channel, sizeof(het_header), (unsigned char *)&het_hdr);
       expv2_uart_send(channel, len, data);
 }

