/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include <string.h>
#define CAN_TRANS_TIMEOUT   (0xFFFF)

typedef struct 
{
  uint32_t   SJW;
  uint32_t   BS1;
  uint32_t   BS2;
  uint32_t	 PreScale;
} CAN_BaudRate_TypeDef;


static const CAN_BaudRate_TypeDef  CAN_BaudRateInitTab[]= 
{
  {CAN_SJW_1TQ, CAN_BS1_6TQ, CAN_BS2_7TQ, 150},   // 20K
  {CAN_SJW_1TQ, CAN_BS1_6TQ, CAN_BS2_7TQ, 60},    // 50K
  {CAN_SJW_1TQ, CAN_BS1_6TQ, CAN_BS2_7TQ, 30},    // 100K
  {CAN_SJW_1TQ, CAN_BS1_9TQ, CAN_BS2_4TQ, 12},    // 250K
  {CAN_SJW_1TQ, CAN_BS1_9TQ, CAN_BS2_4TQ, 6},     // 500K
  {CAN_SJW_1TQ, CAN_BS1_9TQ, CAN_BS2_4TQ, 3},     // 1M
    
};

static BSP_CANx_CallBack  BSP_CAN1_Callback = NULL;
static BSP_CANx_CallBack  BSP_CAN2_Callback = NULL;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 16;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN2_MspInit 1 */
  	HAL_NVIC_SetPriority(CAN2_RX0_IRQn,5,0);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

  /* USER CODE BEGIN CAN2_MspDeInit 1 */
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
static bool __CAN_Init(CAN_HandleTypeDef *hcan, CANBRate_TypeDef brate, uint32_t id)
{
	uint16_t filter_id;
    hcan->Init.Mode = CAN_MODE_NORMAL;                 //回环测试/普�??
    hcan->Init.AutoRetransmission = ENABLE;                 //自动重传 �?要FD

    hcan->Init.Prescaler = CAN_BaudRateInitTab[brate].PreScale;				//分频系数(Fdiv)为brp+1
    hcan->Init.SyncJumpWidth = CAN_BaudRateInitTab[brate].SJW;			//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单�? CAN_SJW_1TQ~CAN_SJW_4TQ
    hcan->Init.TimeSeg1 = CAN_BaudRateInitTab[brate].BS1;					//tbs1范围CAN_BS1_1TQ~CAN_BS1_16TQ
    hcan->Init.TimeSeg2 = CAN_BaudRateInitTab[brate].BS2;					//tbs2范围CAN_BS2_1TQ~CAN_BS2_8TQ
	
    hcan->Init.TimeTriggeredMode = DISABLE;	//非时间触发�?�信模式 
    hcan->Init.AutoBusOff = DISABLE;			//软件自动离线管理
    hcan->Init.AutoWakeUp = DISABLE;			//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP�?)
    hcan->Init.AutoRetransmission = ENABLE;	//禁止报文自动传�?? 
    hcan->Init.ReceiveFifoLocked = DISABLE;	//报文不锁�?,新的覆盖旧的 
    hcan->Init.TransmitFifoPriority = DISABLE;	//优先级由报文标识符决�? 
	
    if(HAL_CAN_Init(hcan) != HAL_OK) 		            //初始化FDCAN
    {
      return false;
    }
	filter_id = id;
    //配置RX滤波�?   
    CAN_FilterTypeDef CANx_RXFilter={0,};
	if(hcan == &hcan1){
		CANx_RXFilter.SlaveStartFilterBank = 0;
	    CANx_RXFilter.FilterBank = 0;
		CANx_RXFilter.FilterMode = CAN_FILTERMODE_IDLIST;//屏蔽位模�?
		CANx_RXFilter.FilterScale = CAN_FILTERSCALE_16BIT;//32�? 
		CANx_RXFilter.FilterIdHigh = (filter_id<<5);//32位ID
		CANx_RXFilter.FilterIdLow = 0;
		CANx_RXFilter.FilterMaskIdHigh = 0;//32位MASK
		CANx_RXFilter.FilterMaskIdLow = 0;
		CANx_RXFilter.FilterActivation = ENABLE;//�?活过滤器
		CANx_RXFilter.FilterFIFOAssignment = CAN_FilterFIFO0;//过滤器关联到FIFO0
	}else{
		CANx_RXFilter.SlaveStartFilterBank = 14;
		CANx_RXFilter.FilterBank = 14;
		CANx_RXFilter.FilterMode = CAN_FILTERMODE_IDMASK;//屏蔽位模�?
		CANx_RXFilter.FilterScale = CAN_FILTERSCALE_32BIT;//32�? 
		CANx_RXFilter.FilterIdHigh = 0x0000;//32位ID
		CANx_RXFilter.FilterIdLow = 0x0000;
		CANx_RXFilter.FilterMaskIdHigh = 0x0000;//32位MASK
		CANx_RXFilter.FilterMaskIdLow = 0x0000;
		CANx_RXFilter.FilterActivation = ENABLE;//�?活过滤器
		CANx_RXFilter.FilterFIFOAssignment = CAN_FilterFIFO0;//过滤器关联到FIFO0
	
	}


    if(HAL_CAN_ConfigFilter(hcan, &CANx_RXFilter) != HAL_OK)//滤波器初始化
    {
      return false;
    }	

    HAL_CAN_Start(hcan);  //�?启CAN
    
    HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);//待处理CAN消息中断
    return true;

}


bool BSP_CAN1_Init(CANBRate_TypeDef brate, uint32_t id)
{
    BSP_CAN1_Callback = NULL;
    hcan1.Instance = CAN1;
    return __CAN_Init(&hcan1, brate, id);
}

bool BSP_CAN1_DeInit( void )
{
  if(HAL_CAN_DeInit(&hcan1) != HAL_OK)
  {
    return false;
  }
  return true;
}

bool  BSP_CAN1_SetRxCallBack(BSP_CANx_CallBack callback)
{
  BSP_CAN1_Callback = callback;
  return true;
}

bool BSP_CAN2_Init(CANBRate_TypeDef brate, uint32_t id)
{
    BSP_CAN2_Callback = NULL;
    hcan2.Instance = CAN2;
    return __CAN_Init(&hcan2, brate, id);
}

bool BSP_CAN2_DeInit( void )
{
  if(HAL_CAN_DeInit(&hcan2) != HAL_OK)
  {
    return false;
  }
  return true;
}

bool  BSP_CAN2_SetRxCallBack(BSP_CANx_CallBack callback)
{
  BSP_CAN2_Callback = callback;
  return true;
}

/**
 * @brief 
 * 
 * @param id 
 * @param pdata 
 * @param length 
 * @return bool 
 */
bool  BSP_CAN1_Transmit(uint16_t id, uint8_t *pdata, uint8_t length)
{
  uint32_t TxMailbox;
  uint32_t timeout = 0;
  CAN_TxHeaderTypeDef tmsg;
  tmsg.StdId = id;
  tmsg.ExtId = 0;
  tmsg.IDE = CAN_ID_STD;
  tmsg.RTR = CAN_RTR_DATA;
  tmsg.DLC = length;
  tmsg.TransmitGlobalTime = DISABLE;

  while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0){
    if(timeout >= CAN_TRANS_TIMEOUT){
      return false;
    }
    timeout++;
  }

  if(HAL_CAN_AddTxMessage(&hcan1, &tmsg, pdata, &TxMailbox) != HAL_OK)//发�??
	{
		return false;
	}
  return true;
}

/**
 * @brief 
 * 
 * @param id 
 * @param pdata 
 * @param length 
 * @return bool 
 */
bool  BSP_CAN2_Transmit(uint16_t id, uint8_t *pdata, uint8_t length)
{
  uint32_t TxMailbox;
  uint32_t timeout = 0;
  CAN_TxHeaderTypeDef tmsg;
  tmsg.StdId = id;
  tmsg.ExtId = 0;
  tmsg.IDE = CAN_ID_STD;
  tmsg.RTR = CAN_RTR_DATA;
  tmsg.DLC = length;
  tmsg.TransmitGlobalTime = DISABLE;
  while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0){
    if(timeout >= CAN_TRANS_TIMEOUT){
      return false;
    }
    timeout++;
  }
  if(HAL_CAN_AddTxMessage(&hcan2, &tmsg, pdata, &TxMailbox) != HAL_OK)//发�??
	{
		return false;
	}
  return true;
}

/**
 * @brief FIFO0回调函数 接收
 * 
 * @param hcan 
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef CANx_RxHeader;
  CANRxMSG_TypeDef rmsg={0,};
	uint8_t rxdata[8];
	if(hcan == &hcan1)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CANx_RxHeader, rxdata);
		
		rmsg.cob_id = CANx_RxHeader.StdId;
		rmsg.length = CANx_RxHeader.DLC;
		memcpy(rmsg.data,rxdata,rmsg.length);
		
		if(BSP_CAN1_Callback)
		{
			BSP_CAN1_Callback(rmsg);
		}
		HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
		
	}
	if(hcan == &hcan2)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CANx_RxHeader, rxdata);
	
		rmsg.cob_id = CANx_RxHeader.StdId;
		rmsg.length = CANx_RxHeader.DLC;
		memcpy(rmsg.data,rxdata,rmsg.length);
		if(BSP_CAN2_Callback)
		{
			BSP_CAN2_Callback(rmsg);
		}
		HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
	}
	
}
/* USER CODE END 1 */
