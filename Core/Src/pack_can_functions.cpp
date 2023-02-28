/*
 * can_functions.cpp
 *
 *  Created on: Jan 9, 2023 / Base | Feb 22, 2023 / CAN FD Added
 *      Author: Andrew Saloutos, Hongmin Kim
 */

#include <pack_can_functions.h>
#include "fdcan.h"

// global variables
extern float dxl_pos_des[9];
extern float dxl_vel_des[9];
extern float dxl_tff_des[9];
extern float dxl_kp[9];
extern float dxl_kd[9];
extern uint8_t tof1[8];
extern uint8_t tof2[8];
extern uint8_t palmTOF;
extern float force1[5];
extern float force2[5];

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
void pack_reply48_joints(uint8_t* fdmsg, float* p, float* v, float* t){
	int p_int[9];
	int v_int[9];
	int t_int[9];

	for (int i=0;i<9;i++){
		p_int[i] = float_to_uint(p[i],P_MIN, P_MAX, 16);
		v_int[i] = float_to_uint(v[i],V_MIN, V_MAX, 12);
		t_int[i] = float_to_uint(t[i]*T_SCALE, -T_MAX, T_MAX, 12);
	}

	int k = 0;
	for (int j=0;j<9;j++){
		fdmsg[k] = p_int[j]>>8;
		fdmsg[k+1] = p_int[j]&0xFF;
		fdmsg[k+2] = v_int[j]>>4;
		fdmsg[k+3] = ((v_int[j]&0xF)<<4) + (t_int[j]>>8);
		fdmsg[k+4] = t_int[j]&0xFF;
		k += 5;
	}
}

void pack_reply48_sens(uint8_t *fdmsg, float * force_data_1, float * force_data_2){
    float fx_temp_1 = fminf(fmaxf(FT_MIN, force_data_1[0]), FT_MAX);
    float fy_temp_1 = fminf(fmaxf(FT_MIN, force_data_1[1]), FT_MAX);
    float fz_temp_1 = fminf(fmaxf(FN_MIN, force_data_1[2]), FN_MAX);
    float theta_temp_1 = fminf(fmaxf(ANG_MIN, force_data_1[3]), ANG_MAX);
    float phi_temp_1 = fminf(fmaxf(ANG_MIN, force_data_1[4]), ANG_MAX);

    float fx_temp_2 = fminf(fmaxf(FT_MIN, force_data_2[0]), FT_MAX);
	float fy_temp_2 = fminf(fmaxf(FT_MIN, force_data_2[1]), FT_MAX);
	float fz_temp_2 = fminf(fmaxf(FN_MIN, force_data_2[2]), FN_MAX);
	float theta_temp_2 = fminf(fmaxf(ANG_MIN, force_data_2[3]), ANG_MAX);
	float phi_temp_2 = fminf(fmaxf(ANG_MIN, force_data_2[4]), ANG_MAX);

    uint16_t fx_int_1 = float_to_uint(fx_temp_1, FT_MIN, FT_MAX, 12);
    uint16_t fy_int_1 = float_to_uint(fy_temp_1, FT_MIN, FT_MAX, 12);
    uint16_t fz_int_1 = float_to_uint(fz_temp_1, FN_MIN, FN_MAX, 12);
    uint16_t theta_int_1 = float_to_uint(theta_temp_1, ANG_MIN, ANG_MAX, 12);
    uint16_t phi_int_1 = float_to_uint(phi_temp_1, ANG_MIN, ANG_MAX, 12);

    uint16_t fx_int_2 = float_to_uint(fx_temp_2, FT_MIN, FT_MAX, 12);
    uint16_t fy_int_2 = float_to_uint(fy_temp_2, FT_MIN, FT_MAX, 12);
    uint16_t fz_int_2 = float_to_uint(fz_temp_2, FN_MIN, FN_MAX, 12);
    uint16_t theta_int_2 = float_to_uint(theta_temp_2, ANG_MIN, ANG_MAX, 12);
    uint16_t phi_int_2 = float_to_uint(phi_temp_2, ANG_MIN, ANG_MAX, 12);

    fdmsg[0] = (fx_int_1>>8);
    fdmsg[1] = fx_int_1&0xFF;
    fdmsg[2] = fy_int_1>>4;
    fdmsg[3] = ((fy_int_1&0x0F)<<4)|(fz_int_1>>8);
    fdmsg[4] = fz_int_1&0xFF;
    fdmsg[5] = theta_int_1>>4;
    fdmsg[6] = ((theta_int_1&0x0F)<<4)|(phi_int_1>>8);
    fdmsg[7] = phi_int_1&0xFF;

    fdmsg[8] = (fx_int_2>>8);
    fdmsg[9] = fx_int_2&0xFF;
    fdmsg[10] = fy_int_2>>4;
    fdmsg[11] = ((fy_int_2&0x0F)<<4)|(fz_int_2>>8);
    fdmsg[12] = fz_int_2&0xFF;
    fdmsg[13] = theta_int_2>>4;
    fdmsg[14] = ((theta_int_2&0x0F)<<4)|(phi_int_2>>8);
    fdmsg[15] = phi_int_2&0xFF;

	fdmsg[16] = tof1[0];
	fdmsg[17] = tof1[1];
	fdmsg[18] = tof1[2];
	fdmsg[19] = tof1[3];
	fdmsg[20] = tof1[4];

	fdmsg[21] = tof2[0];
	fdmsg[22] = tof2[1];
	fdmsg[23] = tof2[2];
	fdmsg[24] = tof2[3];
	fdmsg[25] = tof2[4];

    fdmsg[26] = palmTOF;
}

// new function for just passing through force data from fingertip sensors
void pack_force_reply(uint8_t * msg, float * force_data){

     /// limit data to be within bounds ///
     float fx_temp = fminf(fmaxf(FT_MIN, force_data[0]), FT_MAX);
     float fy_temp = fminf(fmaxf(FT_MIN, force_data[1]), FT_MAX);
     float fz_temp = fminf(fmaxf(FN_MIN, force_data[2]), FN_MAX);
     float theta_temp = fminf(fmaxf(ANG_MIN, force_data[3]), ANG_MAX);
     float phi_temp = fminf(fmaxf(ANG_MIN, force_data[4]), ANG_MAX);
     /// convert floats to unsigned ints ///
     uint16_t fx_int = float_to_uint(fx_temp, FT_MIN, FT_MAX, 12);
     uint16_t fy_int = float_to_uint(fy_temp, FT_MIN, FT_MAX, 12);
     uint16_t fz_int = float_to_uint(fz_temp, FN_MIN, FN_MAX, 12);
     uint16_t theta_int = float_to_uint(theta_temp, ANG_MIN, ANG_MAX, 12);
     uint16_t phi_int = float_to_uint(phi_temp, ANG_MIN, ANG_MAX, 12);
     /// pack ints into the can buffer ///
     msg[0] = (fx_int>>8);
     msg[1] = fx_int&0xFF;
     msg[2] = fy_int>>4;
     msg[3] = ((fy_int&0x0F)<<4)|(fz_int>>8);
     msg[4] = fz_int&0xFF;
     msg[5] = theta_int>>4;
     msg[6] = ((theta_int&0x0F)<<4)|(phi_int>>8);
     msg[7] = phi_int&0xFF;
     }

//// old function for evaluating neural nets on DxL board
//void pack_force_reply(uint8_t * msg, ForceSensor * fs){
//
//     /// limit data to be within bounds ///
//     float fx_temp = fminf(fmaxf(FT_MIN, fs->output_data[0]), FT_MAX);
//     float fy_temp = fminf(fmaxf(FT_MIN, fs->output_data[1]), FT_MAX);
//     float fz_temp = fminf(fmaxf(FN_MIN, fs->output_data[2]), FN_MAX);
//     float theta_temp = fminf(fmaxf(ANG_MIN, fs->output_data[3]), ANG_MAX);
//     float phi_temp = fminf(fmaxf(ANG_MIN, fs->output_data[4]), ANG_MAX);
//     /// convert floats to unsigned ints ///
//     uint16_t fx_int = float_to_uint(fx_temp, FT_MIN, FT_MAX, 12);
//     uint16_t fy_int = float_to_uint(fy_temp, FT_MIN, FT_MAX, 12);
//     uint16_t fz_int = float_to_uint(fz_temp, FN_MIN, FN_MAX, 12);
//     uint16_t theta_int = float_to_uint(theta_temp, ANG_MIN, ANG_MAX, 12);
//     uint16_t phi_int = float_to_uint(phi_temp, ANG_MIN, ANG_MAX, 12);
//     /// pack ints into the can buffer ///
//     msg[0] = (fs->_channel<<4)|(fx_int>>8);
//     msg[1] = fx_int&0xFF;
//     msg[2] = fy_int>>4;
//     msg[3] = ((fy_int&0x0F)<<4)|(fz_int>>8);
//     msg[4] = fz_int&0xFF;
//     msg[5] = theta_int>>4;
//     msg[6] = ((theta_int&0x0F)<<4)|(phi_int>>8);
//     msg[7] = phi_int&0xFF;
//     }


/// ToF Sensor CAN Reply Packet Structure ///
/// 5 x 8bit range measurements
/// CAN packet is 6 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: finger ID (left is 0, right is 1)
/// 1: [tof[7-0]]
/// 2: [tof[7-0]]
/// 3: [tof[7-0]]
/// 4: [tof[7-0]]
/// 5: [tof[7-0]]
/// left finger is sensors 1,2,3,4,5
/// right finger is sensors 6,7,8,9,~

void pack_tof_reply(uint8_t * msg, uint8_t finger){

    /// pack ints into the can buffer ///
    if (finger==0){
		msg[0] = tof1[0];
		msg[1] = tof1[1];
		msg[2] = tof1[2];
		msg[3] = tof1[3];
		msg[4] = tof1[4];
		msg[5] = palmTOF;
    } else if (finger==1){
		msg[0] = tof2[0];
		msg[1] = tof2[1];
		msg[2] = tof2[2];
		msg[3] = tof2[3];
		msg[4] = tof2[4];
    }
}


