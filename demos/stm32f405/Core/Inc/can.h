/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */
typedef enum
{
  CAN_20K = 0,
  CAN_50K = 1,
  CAN_100K = 2,
  CAN_250K = 3,
  CAN_500K = 4,
  CAN_1M   = 5

}CANBRate_TypeDef;

typedef struct __attribute__((__packed__))
{
	uint16_t 	cob_id;
	uint8_t 	length;
	uint8_t		data[8];
}CANRxMSG_TypeDef;

typedef void (*BSP_CANx_CallBack)(CANRxMSG_TypeDef rmsg);
/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */
bool             BSP_CAN1_Init(CANBRate_TypeDef brate, uint32_t id);
bool             BSP_CAN2_Init(CANBRate_TypeDef brate, uint32_t id);
bool             BSP_CAN1_DeInit(void);
bool             BSP_CAN2_DeInit(void);
bool             BSP_CAN1_Transmit(uint16_t ID, uint8_t *pdata, uint8_t length);
bool             BSP_CAN2_Transmit(uint16_t ID, uint8_t *pdata, uint8_t length);
bool             BSP_CAN1_SetRxCallBack(BSP_CANx_CallBack callback);
bool             BSP_CAN2_SetRxCallBack(BSP_CANx_CallBack callback);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

