/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
CAN_TxHeaderTypeDef g_canx_txheader;    /* 发�?�参数句�??? */
CAN_RxHeaderTypeDef g_canx_rxheader;    /* 接收参数句柄 */
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
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  HAL_CAN_Start(&hcan1);
//  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
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
  hcan2.Init.Prescaler = 6;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_5TQ;
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
  HAL_CAN_Start(&hcan2);
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

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
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
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

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
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

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
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5|GPIO_PIN_6);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
uint8_t can1_send_msg(uint32_t id, uint8_t *msg)
{
    uint16_t t = 0;
    uint32_t TxMailbox = CAN_TX_MAILBOX0;

    g_canx_txheader.StdId = id;         /* 标准标识�??? */
    g_canx_txheader.ExtId = id;         /* 扩展标识�???(29�???) */
    g_canx_txheader.IDE = CAN_ID_STD;   /* 使用标准�??? */
    g_canx_txheader.RTR = CAN_RTR_DATA; /* 数据�??? */
    g_canx_txheader.DLC = 8;

    if (HAL_CAN_AddTxMessage(&hcan1, &g_canx_txheader, msg, &TxMailbox) != HAL_OK) /* 发�?�消�??? */
    {
        return 1;
    }

    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3)   /* 等待发�?�完�???,�???有邮箱为�??? */
    {
        t++;

        if (t > 0xFFF)
        {
            HAL_CAN_AbortTxRequest(&hcan1, TxMailbox);     /* 超时，直接中止邮箱的发�?�请�??? */
            return 1;
        }
    }

    return 0;
}

uint8_t can1_receive_msg(uint32_t id, uint8_t *buf)
{
    if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0)     /* 没有接收到数�??? */
    {
        return 0;
    }

    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &g_canx_rxheader, buf) != HAL_OK)  /* 读取数据 */
    {
        return 0;
    }

//    if (g_canx_rxheader.StdId!= id || g_canx_rxheader.IDE != CAN_ID_STD || g_canx_rxheader.RTR != CAN_RTR_DATA)       /* 接收到的ID不对 / 不是标准�??? / 不是数据�??? */
//    {
//        return 0;
//    }

    return g_canx_rxheader.DLC;
}


CAN_RxHeaderTypeDef Rx_pHeader;
/*
 * @brief: CAN Receive Message.
 * @param: "RxData[]" will store the message which has been received, which length must between 0 and 8.
 * @retval: receive status.
 */
uint32_t CAN_RX_Message(uint8_t RxData[])
{

	uint8_t aData[8];    // 缓存接收到的信息

	Rx_pHeader.StdId = 0x000;	// 接收ID（此处无用，can接收�?有的ID号）
	Rx_pHeader.ExtId = 0x0000;
	Rx_pHeader.IDE = CAN_ID_STD;	// 接收标准�?
	Rx_pHeader.DLC = 8;		// 接收8�?8bit数据
	Rx_pHeader.RTR = CAN_RTR_DATA;	// 接收数据�?
	Rx_pHeader.FilterMatchIndex = 0;	// 使用0号过滤器
	Rx_pHeader.Timestamp = 0;

	if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &Rx_pHeader, aData) != HAL_OK)
	{
		return 0;
	}
	else
	{
		// 取出接收到的信息
		for(uint8_t i = 0; i<Rx_pHeader.DLC; i++)
		{
			RxData[i] = aData[i];
		}
		return 1;
	}
}
/* USER CODE BEGIN PV */
uint8_t RxData[8] = {0};    // 缓存接收到的信息
/* USER CODE END PV */
uint8_t can1_test = 0;
uint8_t can1_error = 0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // 判断是哪�?路的CAN发生了中�?
	if(hcan->Instance == CAN1)
	{
		if(CAN_RX_Message(RxData) != 1)
		{
//            // 接收信息失败
//			printf("MCU Received CAN Data ERROR!!!");
//            printf("\n\r");
//			printf("\n\r");
			can1_error++;
		}
		else
		{
			can1_test++;
            // 接受信息成功，处理数�?
//			printf("MCU Received CAN Data: ");
//			for(uint8_t i = 0; i<8; i++)
//			{
//				printf("%d ", RxData[i]);
//			}
//            printf("\n\r");
//			printf("\n\r");
		}
	}
}

uint8_t can2_send_msg(uint32_t id, uint8_t *msg)
{
    uint16_t t = 0;
    uint32_t TxMailbox = CAN_TX_MAILBOX0;

    g_canx_txheader.StdId = id;         /* 标准标识�??? */
    g_canx_txheader.ExtId = id;         /* 扩展标识�???(29�???) */
    g_canx_txheader.IDE = CAN_ID_STD;   /* 使用标准�??? */
    g_canx_txheader.RTR = CAN_RTR_DATA; /* 数据�??? */
    g_canx_txheader.DLC = 8;

    if (HAL_CAN_AddTxMessage(&hcan2, &g_canx_txheader, msg, &TxMailbox) != HAL_OK) /* 发�?�消�??? */
    {
        return 1;
    }

    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) != 3)   /* 等待发�?�完�???,�???有邮箱为�??? */
    {
        t++;

        if (t > 0xFFF)
        {
            HAL_CAN_AbortTxRequest(&hcan2, TxMailbox);     /* 超时，直接中止邮箱的发�?�请�??? */
            return 1;
        }
    }

    return 0;
}

uint8_t can2_receive_msg(uint32_t id, uint8_t *buf)
{
    if (HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0) == 0)     /* 没有接收到数�??? */
    {
        return 0;
    }

    if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &g_canx_rxheader, buf) != HAL_OK)  /* 读取数据 */
    {
        return 0;
    }

    if (g_canx_rxheader.StdId!= id || g_canx_rxheader.IDE != CAN_ID_STD || g_canx_rxheader.RTR != CAN_RTR_DATA)       /* 接收到的ID不对 / 不是标准�??? / 不是数据�??? */
    {
        return 0;
    }

    return g_canx_rxheader.DLC;
}
/* USER CODE END 1 */
