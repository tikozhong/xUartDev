/******************** (C) COPYRIGHT 2015 TIKO *****************************
* File Name          : uartdev.c
* Author             : Tiko Zhong
* Date First Issued  : DEC12,2021
* Description        : This file provides a set of functions needed to manage the
*                      communication using HAL_UARTxxx
********************************************************************************
* History:
* DEC12, 2021: V0.1
	+ 115200bps, 5ms/command test pass

****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _UART_DEV_H
#define _UART_DEV_H

/* Includes ------------------------------------------------------------------*/
#include "misc.h"
#include "ring_buffer.h"

#define UART_TX_BUFF_LEN 32

/* Exported types ------------------------------------------------------------*/
typedef struct{
	UART_HandleTypeDef* huart;
	RINGBUFF_T txRB, rxRB;
	//rx parameter
	u8 *rxPool, *rxBuf0, *rxBuf1, *rxCurBuf, *rxNxtBuf;
	__IO u16 rxPoolLen, rxBufLen;
	//tx parameter
	u8 *txPool;
	__IO u16 txPoolLen;
	u8 txBuff[UART_TX_BUFF_LEN];
	
	u16 flag;
	//callback
	s8 (*beforeSend)(void);
	s8 (*afterSend)(UART_HandleTypeDef *);
}UartRsrc_t;

typedef struct{
	UartRsrc_t rsrc;
	u16 (*TxPolling)	(UartRsrc_t *pRsrc);	// immedially call it 
	u16 (*RxPolling)	(UartRsrc_t *pRsrc);	// per 4ms call it
	u16 (*RxFetchLine)	(UartRsrc_t *pRsrc, char* line, u16 len);	// fetch a line from rxPool
	u16 (*RxFetchFrame)	(UartRsrc_t *pRsrc, u8* frame, u16 frameLen);	// fetch a frame from rxPool
	u16 (*Send)			(UartRsrc_t *pRsrc, const u8* BUF, u16 len);	// send
	void (*SendStr)(UartRsrc_t *pRsrc, const char* FORMAT_ORG, ...);	// send with format
	u16 (*SendFrame)	(UartRsrc_t *pRsrc, const u8* BUF, u16 len);	// send frame
}UartDev_t;

/* Exported variables --------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void setupUartDev(
	UartDev_t *pDev, 
	UART_HandleTypeDef* huart,
	u8* p,				// all rx pool, tx pool, rx double buffer
	u16	rxPoolLen,		// lenth of rx pool, better 512 for 115200bps
	u16 rxBufLen,		// lenth of rx buffer, better >=100 for 115200bps + 4ms polling
	u16 txPoolLen		// lenth of tx pool, must be power(2,n), better 512 for 115200bps
);

#endif /* _MY_UART_H */

/**********************END OF FILE****/
