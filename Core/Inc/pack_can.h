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
void pack_reply48_joints(uint8_t* fdmsg, float* p, float* v, float* t);


#endif /* INC_PACK_CAN_FUNCTIONS_H_ */
