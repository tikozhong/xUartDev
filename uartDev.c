/******************** (C) COPYRIGHT 2015 TIKO *****************************
* File Name          : uartdev.c
* Author             : Tiko Zhong
* Date First Issued  : DEC12,2021
* Description        : This file provides a set of functions needed to manage the
*                      communication using HAL_UARTxxx
********************************************************************************
* History:
* DEC12, 2021: V0.1
	+ 4ms/command test pass

****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "uartDev.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"
#include "crc16.h"
#include "board.h"

extern UART_HandleTypeDef huart2;

/* Public variables ---------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define UART_FRAM_HEAD	(0xed98ba)
#define UART_FRAM_END	(0x89abcd)
/* Private macro -------------------------------------------------------------*/
#define UART_TX_BUFF_LEN	32

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static u16 uartRxMonitor(UartRsrc_t *pRsrc);
static u16 uartRxFetchLine(UartRsrc_t *pRsrc, char* line, u16 len);
static u16 uartRxFetchFrame(UartRsrc_t *pRsrc, u8* frame, u16 frameLen);
static u16 uartTxSend(UartRsrc_t *pRsrc, const u8* BUF, u16 len);
static void uartTxSendString(UartRsrc_t *pRsrc, const char* FORMAT_ORG, ...);
static u16 uartTxSendFrame(UartRsrc_t *pRsrc, const u8* BUF, u16 len);
static u16 uartTxPolling(UartRsrc_t *pRsrc);
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : setupUartDev
* Description    : 
* Input          : 
* Output         : None
* Return         : None
*******************************************************************************/
void setupUartDev(
	UartDev_t *pDev, 
	UART_HandleTypeDef* huart,
	u8* p,				// all rx pool, tx pool, rx double buffer
	u16	rxPoolLen,		// lenth of rx pool, better 512 for 115200bps
	u16 rxBufLen,		// lenth of rx buffer, better >=100 for 115200bps + 4ms polling
	u16 txPoolLen		// lenth of tx pool, must be power(2,n), better 512 for 115200bps
){
	UartRsrc_t *pRsrc = &pDev->rsrc;
	memset(pRsrc,0,sizeof(UartRsrc_t));
	pRsrc->huart = huart;
	
	memset(p,0,(rxPoolLen+2*rxBufLen+txPoolLen));
	pRsrc->rxPool = p;
	pRsrc->rxPoolLen = rxPoolLen;
	pRsrc->rxBuf0 = p+rxPoolLen;
	pRsrc->rxBuf1 = p+rxPoolLen+rxBufLen;
	pRsrc->rxBufLen = rxBufLen;
	
	pRsrc->txPool = p+rxPoolLen+2*rxBufLen;
	pRsrc->txPoolLen = txPoolLen;
	
	pRsrc->rxCurBuf = pRsrc->rxBuf0;
	pRsrc->rxNxtBuf = pRsrc->rxBuf1;
	
	//register op
	pDev->RxFetchLine = uartRxFetchLine;
	pDev->RxFetchFrame = uartRxFetchFrame;
	pDev->Send = uartTxSend;
	pDev->SendStr = uartTxSendString;
	pDev->SendFrame = uartTxSendFrame;
	pDev->TxPolling = uartTxPolling;
	pDev->RxPolling = uartRxMonitor;
	
	RingBuffer_Init(&pRsrc->rxRB, pRsrc->rxPool, 1, pRsrc->rxPoolLen);
	RingBuffer_Init(&pRsrc->txRB, pRsrc->txPool, 1, pRsrc->txPoolLen);	
	
	//start to receive
	while(HAL_UART_Receive_IT(huart, pRsrc->rxCurBuf, pRsrc->rxBufLen) == HAL_OK){}
}

static u16 uartTxSend(UartRsrc_t *pRsrc, const u8* BUF, u16 len){
	u16 sentBytes;
	if(BUF == NULL || len==0)	return 0;
	for(sentBytes=0;1;){
		sentBytes += RingBuffer_InsertMult(&pRsrc->txRB, &BUF[sentBytes], len-sentBytes);
		if(sentBytes>=len){	break;	}
		uartTxPolling(pRsrc);
	}
	return sentBytes;
}

static void uartTxSendString(UartRsrc_t *pRsrc, const char* FORMAT_ORG, ...){
	va_list ap;
	s16 bytes;
	char buff[MAX_CMD_LEN] = {0};
	
	if(FORMAT_ORG == NULL)	return ;
	//take string
	va_start(ap, FORMAT_ORG);
	bytes = vsnprintf(buff, MAX_CMD_LEN, FORMAT_ORG, ap);
	va_end(ap);	
	uartTxSend(pRsrc,(u8*)buff,bytes);
}

static u16 uartTxSendFrame(UartRsrc_t *pRsrc, const u8* BUF, u16 len){
	u16 crc;
	u8 buff[5];
	
	if(BUF == NULL || len==0)	return 0;

	crc = CRC16(BUF, len, 0xacca);
	buff[0] = UART_FRAM_HEAD&0xff;
	buff[1] = (UART_FRAM_HEAD>>8)&0xff;	
	buff[2] = (UART_FRAM_HEAD>>16)&0xff;
	uartTxSend(pRsrc,buff,3);
	uartTxSend(pRsrc,BUF,len);
	buff[0] = crc & 0xff;
	buff[1] = (crc>>8) & 0xff;
	buff[2] = UART_FRAM_END&0xff;
	buff[3] = (UART_FRAM_END>>8)&0xff;
	buff[4] = (UART_FRAM_END>>16)&0xff;
	uartTxSend(pRsrc,buff,5);

	return (len+8);
}

static u16 uartRxMonitor(UartRsrc_t *pRsrc){
	u16 bytes,bytesReceived;
	u8* pTmp;
	UART_HandleTypeDef *huart = pRsrc->huart;
	
	__HAL_UART_DISABLE_IT(pRsrc->huart, UART_IT_RXNE);
	bytesReceived = huart->RxXferSize - huart->RxXferCount;
	if(bytesReceived==0){
		__HAL_UART_ENABLE_IT(pRsrc->huart, UART_IT_RXNE);
		return 0;
	}
	//restart uart
	huart->pRxBuffPtr  = pRsrc->rxNxtBuf;
	huart->RxXferCount = pRsrc->rxBufLen;
	__HAL_UART_ENABLE_IT(pRsrc->huart, UART_IT_RXNE);

	bytes = RingBuffer_InsertMult(&pRsrc->rxRB, pRsrc->rxCurBuf, bytesReceived);
	if(bytes<bytesReceived)		pRsrc->flag |= BIT(0);
	
	pTmp = pRsrc->rxCurBuf;
	pRsrc->rxCurBuf = pRsrc->rxNxtBuf;
	pRsrc->rxNxtBuf = pTmp;
	return bytes;
}

//better if len equ rxRB' pool len
static u16 uartRxFetchLine(UartRsrc_t *pRsrc, char* line, u16 len){
	u8 ret = 0;
	char *p, *buff = line;	//buff[MAX_CMD_LEN] = {0};
	u16 i,j,tmp;
	s16 lineLen=0, bytes, count;
	
	bytes = RingBuffer_PopMult(&pRsrc->rxRB, buff, len);
	if(bytes<=0)	return 0;
	RingBuffer_Flush(&pRsrc->rxRB);
	
	for(i=0;i<bytes;i++){
		p = strstr(&buff[i], CMD_END);
		if(p){
			lineLen = p-(char*)&buff[i]+strlen(CMD_END);
			memmove(line, &buff[i], (lineLen>len?len:lineLen));
			count = bytes - (i+lineLen);
			if(count > 0){
				RingBuffer_InsertMult(&pRsrc->rxRB, &buff[i+lineLen], count);
			}
			line[lineLen] = 0;
			ret = 1;
			break;
		}
	}
	
	if(p==NULL){	RingBuffer_InsertMult(&pRsrc->rxRB,buff,bytes);	}	

	return lineLen;
}

//better if frameLen equ rxRB' pool len
static u16 uartRxFetchFrame(UartRsrc_t *pRsrc, u8* frame, u16 frameLen){
	u16 i,j,crc0,crc1;
	u8 *head, *end, *pCrc, *buff = frame;
	u16 len = 0;
	s16 bytes,count;
	
	bytes = RingBuffer_PopMult(&pRsrc->rxRB, buff, frameLen);
	if(bytes<=0)	return 0;
	RingBuffer_Flush(&pRsrc->rxRB);
	
	head = NULL;
	for(i=0;(i+2)<bytes;i++){
		if(	(buff[i+0] == (UART_FRAM_HEAD & 0XFF)) &&
			(buff[i+1] == ((UART_FRAM_HEAD>>8) & 0XFF)) &&
			(buff[i+2] == ((UART_FRAM_HEAD>>16) & 0XFF))
		){
			head = &buff[i];
			j = i+3+2;
			break;
		}
	}
	if(head==NULL){
		RingBuffer_InsertMult(&pRsrc->rxRB, buff, bytes);
		return 0;
	}
	
	end = NULL;
	for(i=j;(i+2)<bytes;i++){
		if(	(buff[i+0] == (UART_FRAM_END & 0XFF)) &&
			(buff[i+1] == ((UART_FRAM_END>>8) & 0XFF)) &&
			(buff[i+2] == ((UART_FRAM_END>>16) & 0XFF))
		){
			end = &buff[i];
			break;
		}
	}
	if(end==NULL){
		RingBuffer_InsertMult(&pRsrc->rxRB, buff, bytes);
		return 0;
	}

	count = &buff[bytes]-(end+3);
	if(count>0){	RingBuffer_InsertMult(&pRsrc->rxRB, end+3, count);	}
	
	pCrc = end-1;
	crc0 = *pCrc;
	crc0 <<= 8;
	pCrc--;
	crc0 |= *pCrc;
	len = end-head-3-2;
	crc1 = CRC16(head+3,len,0xacca);
	if(crc0==crc1){
		memmove(frame, head+3, (len>=frameLen?frameLen:len));
		frame[len]=0;
		return len;
	}

	return 0;
}

static u16 uartTxPolling(UartRsrc_t *pRsrc){
	s32 count;
	u16 bytes;
	
	if(RingBuffer_IsEmpty(&pRsrc->txRB))	return 0;
	if(pRsrc->huart->gState != HAL_UART_STATE_READY)	return 0;
	if(pRsrc->beforeSend){
		if(pRsrc->beforeSend()<0)	return 0;
	}
	
	bytes = RingBuffer_PopMult(&pRsrc->txRB, pRsrc->txBuff, UART_TX_BUFF_LEN);
	HAL_UART_Transmit_IT(pRsrc->huart, pRsrc->txBuff, bytes);
	return bytes;
}

/**********************END OF FILE****/
