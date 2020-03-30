/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SHAKE_TRES  5	//amount of shaking IRQ's to wake up from DROWSE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
CAN_FilterTypeDef canFilterConfig;

BtModuleTypeDef bt;
uint8_t len = 6;
//BtTxMsgTypeDef TxMes;
BtRxMsgTypeDef RxMes;

uint8_t tx_buf[4] = {0x01, 0xDF, 0xC5, 0xD1};
uint8_t rx_buf[8] = {0};
uint8_t rx_buf_FMI[8][8] = {0};
//uint8_t flag = 0;
uint32_t freq_a, freq_b;
uint32_t TxMailbox;
HAL_StatusTypeDef TxStatus;
uint16_t uSendLimit;
uint16_t can_rx_counter = 0;
uint32_t can_sleep = 0;

volatile uint8_t shake_int = 0;
typedef enum {
    Sleep = 0x00, //
		toSleep = 0x01,
		Wake = 0x02,
    toWake = 0x03,  //
		Unknown = 0x04
} POWER_STATE;
POWER_STATE pwr_state = Wake;
uint32_t freq_a, freq_b, sysclock, pclk1, pclk2;
RCC_OscInitTypeDef ocs_struct;
typedef struct 
{
	uint16_t can_rx;
	uint16_t can_rx_insec;
	uint8_t wakeup;
	uint16_t can_error;
}counter_t;
counter_t counter;
typedef struct 
{
	uint8_t can_rx;
}FlagTypeDef;
FlagTypeDef flag;

MMSCommandsTypeDef mms;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void can_user_settings(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_CAN_RxFIFO0MsgPendingCallback(CAN_HandleTypeDef* hcan);
void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin== GPIO_PIN_0)
	{
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		if (shake_int < SHAKE_TRES) {	shake_int++;}
		else
		{
			shake_int = 0;
			if (pwr_state == Sleep) 		{pwr_state = toWake;}
			else if (pwr_state == Wake)	{pwr_state = toSleep;}
		}
	}
}
void HAL_CAN_RxFIFO0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rx_buf) != HAL_OK)
	{
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	for (uint8_t i = 0; i < RxHeader.DLC; i++)
	{
		rx_buf_FMI[RxHeader.FilterMatchIndex][i] = rx_buf[i];
	}
	counter.can_rx++;
	mms.Source = ((rx_buf[1]&0x01)<<1) + ((rx_buf[0]&0x80)>>7);
	mms.BtnCallAnsw = (rx_buf[1]&0x02)>>1;
	mms.BtnCallRej = 	(rx_buf[1]&0x04)>>2;
	mms.BtnPrev = 		(rx_buf[0]&0x40)>>6;
	mms.BtnNext = 		(rx_buf[0]&0x20)>>5;
	mms.BtnMuteOff = 	(rx_buf[0]&0x10)>>4;
	mms.BtnMuteOn = 	(rx_buf[0]&0x08)>>3;
	mms.BtnMuteTgl = 	(rx_buf[0]&0x04)>>2;
	mms.BtnVolMinus = (rx_buf[0]&0x02)>>1;
	mms.BtnVolPlus = 	(rx_buf[1]&0x01);
	flag.can_rx = 1;
}
void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan)
{
	counter.wakeup++;
	HAL_CAN_WakeUp(hcan);
	can_sleep = HAL_CAN_IsSleepActive(hcan);	//don*t wakeup without this status request
}
/*void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan)
{
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}*/
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	//HAL_PWR_DisableSleepOnExit();
	HAL_CAN_GetError(hcan);
	HAL_CAN_ResetError(hcan);
	counter.can_error++;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		HAL_UART_Receive_IT(&huart2, RxMes.short_resp, len);
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{

}
void canToUartCommands(MMSCommandsTypeDef* mms_t)
{
	/*if (mms_t->Source == 0x02)	//BT is on
	{*/
		if (mms_t->BtnCallAnsw)
		{
			mms_t->BtnCallAnsw = 0;
			
		}			
		if(mms_t->BtnCallRej)
		{
			mms_t->BtnCallRej = 0;
			
		}
		if (mms_t->BtnPrev)
		{
			mms_t->BtnPrev = 0;
			
		}
		if (mms_t->BtnNext)
		{
			mms_t->BtnNext = 0;
			
		}
		if (mms_t->BtnMuteTgl)
		{
			mms_t->BtnMuteTgl = 0;
			static uint8_t trig = 1;
			if (trig^=1)	{mms_t->BtnMuteOff = 1;}
			else					{mms_t->BtnMuteOn = 1;}
		}
		if (mms_t->BtnMuteOff)
		{
			mms_t->BtnMuteOff = 0;
			
		}
		if (mms_t->BtnMuteOn)
		{
			mms_t->BtnMuteOn = 0;
		
		}
		if (mms_t->BtnVolMinus)
		{
			mms_t->BtnVolMinus = 0;
			
		}
		if (mms_t->BtnVolPlus)
		{
			mms_t->BtnVolPlus = 0;
		
		}
	/*}
	else if (mms_t->Source == 0x03)	//FM is on
	{
	
	}*/
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void can_user_settings(void)
{
	canFilterConfig.FilterBank = 0;	/*!< Specifies the filter which will be initialized. 
                                       This parameter must be a number between Min_Data = 0 and Max_Data = 13. */
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;	//CAN_FILTERMODE_IDLIST or CAN_FILTERMODE_IDMASK
  canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;	//CAN_FILTERSCALE_16BIT or CAN_FILTERSCALE_32BIT 
  canFilterConfig.FilterIdHigh = 0x04CE << 5;	// идентификатор №1
  canFilterConfig.FilterIdLow = 0x0000;	// идентификатор №2
  canFilterConfig.FilterMaskIdHigh = 0x07FF << 5;	// идентификатор №3
  canFilterConfig.FilterMaskIdLow = 0x0000;	// идентификатор №4
  canFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canFilterConfig.FilterActivation = ENABLE;
  /*canFilterConfig.BankNumber = 0;*/	/*!< Select the start slave bank filter
                                       This parameter must be a number between Min_Data = 0 and Max_Data = 28. */ 
	HAL_CAN_ConfigFilter(&hcan, &canFilterConfig);
	
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_ERROR_WARNING|CAN_IT_ERROR_PASSIVE|CAN_IT_BUSOFF|CAN_IT_LAST_ERROR_CODE|CAN_IT_ERROR);
	HAL_CAN_RegisterCallback(&hcan, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, HAL_CAN_RxFIFO0MsgPendingCallback);
	
	HAL_CAN_ActivateNotification(&hcan,CAN_IT_WAKEUP);
	HAL_CAN_RegisterCallback(&hcan, HAL_CAN_WAKEUP_FROM_RX_MSG_CB_ID, HAL_CAN_WakeUpFromRxMsgCallback);
	
	
	TxHeader.StdId = 0x0532;             //
	TxHeader.ExtId = 0x00;                          //
	TxHeader.IDE = CAN_ID_STD ;                 // 
	TxHeader.RTR = CAN_RTR_DATA;                    // 
	TxHeader.DLC = 8;  
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	mms.p[0] = &mms.Source;
	mms.p[1] = &mms.BtnPrev;
	mms.p[2] = &mms.BtnNext;
	mms.p[3] = &mms.BtnMuteOff;
	mms.p[4] = &mms.BtnMuteOn;
	mms.p[5] = &mms.BtnMuteTgl;
	mms.p[6] = &mms.BtnVolMinus;
	mms.p[7] = &mms.BtnVolPlus;
	mms.p[8] = &mms.BtnCallAnsw;
	mms.p[9] = &mms.BtnCallRej;
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart2, RxMes.short_resp, len);
	can_user_settings();
	HAL_CAN_Start(&hcan);
	HAL_DBGMCU_EnableDBGSleepMode();	//DBGMCU->CR |= DBGMCU_CR_DBG_SLEEP;
	//HAL_DBGMCU_EnableDBGStopMode();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	freq_a = HAL_RCC_GetHCLKFreq();
  while (1)
  {
		if (flag.can_rx)
		{
			flag.can_rx = 0;
			canToUartCommands(&mms);
		}
		if (pwr_state == toSleep)
		{
			//HAL_CAN_RequestSleep(&hcan);
			pwr_state = Sleep;
			
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			/* Suspend Tick increment to prevent wakeup by Systick interrupt.
     Otherwise the Systick interrupt will wake up the device within 1ms (HAL time base) */
			HAL_SuspendTick();
			__HAL_RCC_PWR_CLK_ENABLE();
			//@arg PWR_MAINREGULATOR_ON: Stop mode with regulator ON
			//@arg PWR_LOWPOWERREGULATOR_ON: Stop mode with low power regulator ON
			//HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
			HAL_PWR_EnterSLEEPMode(0, PWR_SLEEPENTRY_WFI); //__WFI();
			HAL_NVIC_DisableIRQ(EXTI0_IRQn);
			HAL_ResumeTick();
			//-HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			freq_a = HAL_RCC_GetHCLKFreq();
			//SystemClock_Config();
			freq_b = HAL_RCC_GetHCLKFreq();
			pwr_state = Wake;
		}

		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		//freq_b = HAL_RCC_GetHCLKFreq();       
		sysclock = HAL_RCC_GetSysClockFreq();
		pclk1 = HAL_RCC_GetPCLK1Freq();
    pclk2 = HAL_RCC_GetPCLK2Freq();
		HAL_RCC_GetOscConfig(&ocs_struct);
		HAL_Delay(500);
		HAL_NVIC_EnableIRQ(EXTI0_IRQn);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		can_sleep = HAL_CAN_IsSleepActive(&hcan);
		if (!can_sleep)
		{
			if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 0)
			{
				uSendLimit = 0xFF;
				do 
				{
					TxStatus = HAL_CAN_AddTxMessage(&hcan, &TxHeader, tx_buf, &TxMailbox);
				}
				while (TxStatus != HAL_OK && uSendLimit--);
				tx_buf[0]++;
			}
		}
		HAL_Delay(500);
		counter.can_rx_insec = counter.can_rx;
		counter.can_rx = 0;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 PA5 PA6 
                           PA7 PA8 PA9 PA10 
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB11 PB12 PB13 PB14 
                           PB15 PB3 PB4 PB5 
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
