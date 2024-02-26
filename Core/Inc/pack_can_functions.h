/*
 * can_functions.h
 *
 *  Created on: Jan 9, 2023
 *      Author: saloutos
 */

#ifndef INC_PACK_CAN_FUNCTIONS_H_
#define INC_PACK_CAN_FUNCTIONS_H_


#include "main.h"
#include "math_ops.h"

// prototypes
void pack_reply(uint8_t *msg, int dxl_id, float p, float v, float t);
void pack_force_reply(uint8_t * msg, float * force_data);
void pack_reply48_joints(uint8_t* fdmsg, float* p, float* v, float* t);
void pack_reply48_sens(uint8_t *fdmsg, float * force_data_1, float * force_data_2);
//void pack_force_reply(uint8_t * msg, ForceSensor * fs);
void pack_tof_reply(uint8_t * msg, uint8_t finger);


#endif /* INC_PACK_CAN_FUNCTIONS_H_ */
