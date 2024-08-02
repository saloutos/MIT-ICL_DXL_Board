/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;

extern FDCAN_HandleTypeDef hfdcan2;

/* USER CODE BEGIN Private defines */
//MAIN BUS CAN
#define ENABLE_COMMAND			1
#define	TX_JOINTS				2
// TODO: clearer names for these
#define COMMAND_BUS_1			3
#define COMMAND_BUS_2			4

// mode select messages
#define MS_DISABLE				0xFA
#define MS_MOTOR_DEBUG			0xFB
#define MS_CUR_CTRL				0xFC
#define MS_POS_CTRL				0xFD

/// Value Limits ///
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -65.0f
#define V_MAX 65.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 10.0f
#define T_MIN -72.0f
#define T_MAX 72.0f

#define KP_SCALE 50.0f
#define KD_SCALE 50.0f
#define T_SCALE 50.0f


extern FDCAN_RxHeaderTypeDef rxMsg_sys;
extern uint8_t sys_rx_buf[48];
/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);
void MX_FDCAN2_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

