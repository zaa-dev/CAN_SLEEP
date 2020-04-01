/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define CYCLE_BUF_SIZE_x 8
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define BT_ON 			"COM+PWOS\r\n"
#define BT_OFF 			"COM+PWDS\r\n"
#define BT_RES 			"COM+REBOOT\r\n"
#define BT_VOL_P		"COM+VP\r\n"
#define BT_VOL_M		"COM+VD\r\n"
#define BT_VOL_Q		"COM+GV\r\n"
#define BT_MODE_BT	"COM+MBT\r\n"
#define BT_MODE_Q		"COM+IQ\r\n"
#define	BT_PAIR			"BT+PR\r\n"
#define	BT_CON_LAST	"BT+AC\r\n"
#define	BT_DISCON		"BT+DC\r\n"
#define BT_ANSW_CALL		"BT+CA\r\n"
#define BT_REF_CALL 		"BT+CJ\r\n"
#define BT_HANGUP_CALL	"BT+CE\r\n"
#define BT_REDIAL		"BT+CR\r\n"

#define BT_STATUS_QUERY	"COM+IQ\r\n"
#define BT_VOLUME_QUERY "COM+GV\r\n"
#define BT_NEXT_TRACK		"COM+PN\r\n"
#define BT_PREV_TRACK		"COM+PV\r\n"
#define BT_PLAY_MUSIC		"COM+PA\r\n"
#define BT_PAUS_MUSIC		"COM+PU\r\n"

//char *c[]={"COM+PWOS\r\n","COM+PWDS\r\n","COM+REBOOT\r\n"};
static const char bt_commands [][15] = {
{0},BT_ON,	BT_OFF,	BT_RES, BT_VOL_P, BT_VOL_M, BT_VOL_Q, BT_MODE_BT, BT_MODE_Q, BT_PAIR, BT_CON_LAST, BT_DISCON, BT_ANSW_CALL, BT_REF_CALL, BT_HANGUP_CALL, BT_REDIAL,
BT_STATUS_QUERY, BT_VOLUME_QUERY, BT_NEXT_TRACK, BT_PREV_TRACK, BT_PLAY_MUSIC, BT_PAUS_MUSIC,
};
typedef enum
{
	toNop,
	toOn,
	toOff,
	toRst,
	toVolUp,
	toVolDn,
	toVolQ,
	toModeBt,
	toModeQ,
	toPair,
	toConLst,
	toDiscon,
	toAnswCall,
	toRefCall,
	toHgUp,
	toRedial,
	
	quStatus,
	quVol,
	toNextTrack,
	toPrevTrack,
	toPlayTrack,
	toPausTrack
} BtCommandTypeDef;
typedef enum
{
	btNoCon,
	btOCon,
	btOff
} BtStatusTypeDef;
typedef struct
{
  uint8_t short_resp[8];     
	uint8_t long_resp[32]; 
}BtRxMsgTypeDef;
/*typedef struct
{
	BtCommandTypeDef command;
	
	BtRxMsgTypeDef*            pBtRxMsg;     //!< Pointer to receive structure  
	//BtTxMsgTypeDef*            pBtTxMsg;     //!< Pointer to transmit structure  
	uint8_t rx;
} BtModuleTypeDef;*/
typedef struct
{
	BtCommandTypeDef command;
	BtCommandTypeDef command_buf[8];
	uint8_t var;
	uint8_t rx_buf[10];
	BtRxMsgTypeDef*            pBtRxMsg;     /*!< Pointer to receive structure  */
	//BtTxMsgTypeDef*            pBtTxMsg;     /*!< Pointer to transmit structure  */
	uint32_t ticks;
} BtModuleTypeDef;
typedef struct 
{
	uint8_t Source;
	uint8_t BtnPrev;
	uint8_t BtnNext;
	uint8_t BtnMuteOff;
	uint8_t BtnMuteOn;
	uint8_t BtnMuteTgl;
	uint8_t BtnVolMinus;
	uint8_t BtnVolPlus;
	uint8_t BtnCallAnsw;
	uint8_t BtnCallRej;
	uint8_t *p[10];
} MMSCommandsTypeDef;

typedef struct
{
	BtCommandTypeDef buffer[CYCLE_BUF_SIZE_x];
	uint16_t idxIn;
  uint16_t idxOut;
} RingBufTypeDef;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void RING_Put(BtCommandTypeDef btCommand/*uint8_t* array*/, RingBufTypeDef* buf);
BtCommandTypeDef RING_Pop(RingBufTypeDef *buf);
uint8_t RING_GetCount(RingBufTypeDef *buf);
int32_t RING_ShowSymbol(uint16_t symbolNumber ,RingBufTypeDef *buf);
void RING_Clear(RingBufTypeDef* buf);
void RING_Init(RingBufTypeDef *buf);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
