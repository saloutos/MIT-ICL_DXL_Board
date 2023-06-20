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
#define CAN_TX_DXL9            0
#define CAN_TX_DXL1            1 // commands from controller to DXLs
#define CAN_TX_DXL2            2
#define CAN_TX_DXL3            3
#define CAN_TX_DXL4            4
#define CAN_TX_DXL5            5
#define CAN_TX_DXL6            6
#define CAN_TX_DXL7            7
#define CAN_TX_DXL8            8

#define CAN_FORCE_1            9
#define CAN_FORCE_2            10
#define CAN_TOF_1              11
#define CAN_TOF_2              12
#define CAN_TOF_PALM           22

#define CAN_RX_DXL1            13 // responses from DXLs to controller
#define CAN_RX_DXL2            14
#define CAN_RX_DXL3            15
#define CAN_RX_DXL4            16
#define CAN_RX_DXL5            17
#define CAN_RX_DXL6            18
#define CAN_RX_DXL7            19
#define CAN_RX_DXL8            20
#define CAN_RX_DXL9            21

//SENSOR BUS CAN
#define CAN2_FORCE_1            5
#define CAN2_FORCE_2            6
#define CAN2_TOF_1              7
#define CAN2_TOF_2              8
#define CAN2_TOF_PALM           10

// new sensor IDs
#define CAN2_PHAL_1				11
#define CAN2_PHAL_2				12
#define CAN2_PHAL_3				13
#define CAN2_PHAL_4				14
// pressure sensors
#define CAN2_RAW_BMP_1			15
#define CAN2_RAW_BMP_2			16
#define CAN2_RAW_BMP_3			17
#define CAN2_RAW_BMP_4			18

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

// new limits for force sensors, need to test these!
#define FT_MIN -20.0f
#define FT_MAX 20.0f
#define FN_MIN -30.0f
#define FN_MAX 30.0f
#define ANG_MIN -135.0f
#define ANG_MAX 45.0f
#define RNG_MAX 255 // this probably won't be necessary

extern FDCAN_RxHeaderTypeDef rxMsg_sys;
extern FDCAN_RxHeaderTypeDef rxMsg_sense;
extern uint8_t sys_rx_buf[48];
extern uint8_t sense_rx_buf[8];
/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);
void MX_FDCAN2_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

