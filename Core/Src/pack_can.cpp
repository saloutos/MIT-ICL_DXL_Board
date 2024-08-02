/*
 * can_functions.cpp
 *
 *  Created on: Jan 9, 2023 / Base | Feb 22, 2023 / CAN FD Added
 *      Author: Andrew Saloutos, Hongmin Kim
 */

#include <pack_can.h>
#include "fdcan.h"

/// CAN Reply Packet Structure ///
/// 16 bit position, between -4*pi and 4*pi
/// 12 bit velocity, between -30 and + 30 rad/s
/// 12 bit current, between -40 and 40;
/// CAN Packet is 5 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]]
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], current[11-8]]
/// 4: [current[7-0]]
void pack_reply(uint8_t *msg, int dxl_id, float p, float v, float t){
    int p_int = float_to_uint(p, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(v, V_MIN, V_MAX, 12);
    int t_int = float_to_uint(t*T_SCALE, -T_MAX, T_MAX, 12);
    msg[0] = dxl_id;
    msg[1] = p_int>>8;
    msg[2] = p_int&0xFF;
    msg[3] = v_int>>4;
    msg[4] = ((v_int&0xF)<<4) + (t_int>>8);
    msg[5] = t_int&0xFF;
    }

/// CAN FD Reply Packet Structure ///
// 8 typical CAN packets in a row, 5*8=40bytes
void pack_reply48_joints(uint8_t* fdmsg, float* p, float* v, float* t){
	int p_int[8];
	int v_int[8];
	int t_int[8];

	// now, only 5 motors -> 25 of 48 bytes are used
	for (int i=0; i<5; i++){
		p_int[i] = float_to_uint(p[i],P_MIN, P_MAX, 16);
		v_int[i] = float_to_uint(v[i],V_MIN, V_MAX, 12);
		t_int[i] = float_to_uint(t[i]*T_SCALE, -T_MAX, T_MAX, 12);
	}
	int k = 0;
	for (int j=0; j<5; j++){
		fdmsg[k] = p_int[j]>>8;
		fdmsg[k+1] = p_int[j]&0xFF;
		fdmsg[k+2] = v_int[j]>>4;
		fdmsg[k+3] = ((v_int[j]&0xF)<<4) + (t_int[j]>>8);
		fdmsg[k+4] = t_int[j]&0xFF;
		k += 5;
	}
}
