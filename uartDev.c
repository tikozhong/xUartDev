/******************** (C) COPYRIGHT 2015 INCUBECN *****************************
* File Name          : uartdev.c
* Author             : Tiko Zhong
* Date First Issued  : 04/20/2020
* Description        : This file provides a set of functions needed to manage the
*                      communication using HAL_UARTxxx
********************************************************************************
* History:
* 04/20/2020: V0.1
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "uartDev.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"
#include "crc16.h"
#include "board.h"
/* Public variables ---------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define UART_FRAM_HEAD	(0xed98ba)
#define UART_FRAM_END	(0x89abcd)
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static u16 uartTxPolling(UartRsrc_t *pRsrc);
static u8 uartRxMonitor(UartRsrc_t *pRsrc);
static u16 uartRxFetchLine(UartRsrc_t *pRsrc, char* line, u16 len);
static u16 uartRxFetchFrame(UartRsrc_t *pRsrc, u8* frame, u16 frameLen);
static u16 uartTxSendFrame(UartRsrc_t *pRsrc, const u8* BUF, u16 len);
static s16 uartSend(UartRsrc_t *pRsrc, const u8* BUF, u16 len);

#if	UART_ALL_FUNCTION
static void uartTxSendString(UartRsrc_t *pRsrc, const char* FORMAT_ORG, ...);
static s16 uartSendSync(UartRsrc_t *pRsrc, const u8* BUF, u16 len);
static void uartTxSendStringSync(UartRsrc_t *pRsrc, const char* FORMAT_ORG, ...);
#endif
static s16 TjcSendCmd(UartRsrc_t* p, const char* FORMAT_ORG, ...);
static void uartStartRecv(UartRsrc_t *pRsrc);
static u8 uartTestRestartRecv(UartRsrc_t *pRsrc);
/*******************************************************************************
* Function Name  : uartSrscSetup
* Description    : 
* Input          : 
* Output         : None
* Return         : None
*******************************************************************************/
void setupUartDev(
	UartDev_t *pDev, 
	UART_HandleTypeDef* huart,
	u8* txPool, u16 txPoolLen,
	u8* rxPool,	u16	rxPoolLen,
	u8* rxDoubleBuff,	u16 rxBufLen
){
	UartRsrc_t *pRsrc = &pDev->rsrc;
	pRsrc->huart = huart;
	
	pRsrc->rxPool = rxPool;
	pRsrc->rxPoolLen = rxPoolLen;
	
	pRsrc->rxBuf0 = rxDoubleBuff;
	pRsrc->rxBuf1 = rxDoubleBuff + rxBufLen;
	pRsrc->rxBufLen = rxBufLen;
	
	pRsrc->txPool = txPool;
	pRsrc->txPoolLen = txPoolLen;

	pRsrc->rxCurBuf = pRsrc->rxBuf0;
	pRsrc->rxNxtBuf = pRsrc->rxBuf1;
	
	pRsrc->afterSend = NULL;
	pRsrc->beforeSend = NULL;
	
	//register op
	pDev->TxPolling = uartTxPolling;
	pDev->RxPolling = uartRxMonitor;
	pDev->RxFetchLine = uartRxFetchLine;
	pDev->RxFetchFrame = uartRxFetchFrame;
	
	pDev->TxSendFrame = uartTxSendFrame;
	pDev->Send = uartSend;
	pDev->TJC_Send = TjcSendCmd;

#if	UART_ALL_FUNCTION
	pDev->SendSync = uartSendSync;
	pDev->SendStr = uartTxSendString;
	pDev->SendStrSync = uartTxSendStringSync;
#endif

	pDev->StartRcv = uartStartRecv;
	pDev->TestRestartRcv = uartTestRestartRecv;
	
	RingBuffer_Init(&pRsrc->txRB, pRsrc->txPool, 1, pRsrc->txPoolLen);
}

static void uartStartRecv(UartRsrc_t *pRsrc){
	memset(pRsrc->rxBuf0,0,pRsrc->rxBufLen*2);
	memset(pRsrc->rxPool,0,pRsrc->rxPoolLen);
	RingBuffer_Init(&pRsrc->rxRB, pRsrc->rxPool, 1, pRsrc->rxPoolLen);	
	pRsrc->rxCurBuf = pRsrc->rxBuf0;
	pRsrc->rxNxtBuf = pRsrc->rxBuf1;
	while(HAL_UART_Receive_IT(pRsrc->huart, pRsrc->rxCurBuf, pRsrc->rxBufLen) != HAL_OK){
	}
}

static u8 uartTestRestartRecv(UartRsrc_t *pRsrc){
	// auto start and restart
	if((pRsrc->huart->RxState & BIT(1)) == 0){
		HAL_UART_AbortReceive_IT(pRsrc->huart);
		uartStartRecv(pRsrc);
		return 1;
	}
	return 0;
}

static u16 uartTxPolling(UartRsrc_t *pRsrc){
	s32 bytes;
	
	if(RingBuffer_IsEmpty(&pRsrc->txRB))	return 0;	
	if(pRsrc->huart->gState & BIT(0))	return 0;
	if(pRsrc->beforeSend && pRsrc->beforeSend()<0)	return 0;
	
	bytes = RingBuffer_PopMult(&pRsrc->txRB, (u8*)pRsrc->txBuff, UART_TX_BUFF_LEN);
	
	if(bytes>0){
		while(HAL_UART_Transmit_IT(pRsrc->huart, pRsrc->txBuff, bytes) != HAL_OK){}
	}
	return bytes;
}

static s16 uartSend(UartRsrc_t *pRsrc, const u8* BUF, u16 len){
	u16 sentBytes;
	if(BUF == NULL || len==0)	return 0;
	for(sentBytes = 0; sentBytes < len; ){
		sentBytes += RingBuffer_InsertMult(&pRsrc->txRB, (void*)&BUF[sentBytes], len-sentBytes);
		uartTxPolling(pRsrc);
	}
	return sentBytes;
}

#if	UART_ALL_FUNCTION
static s16 uartSendSync(UartRsrc_t *pRsrc, const u8* BUF, u16 len){
	u16 sentBytes;
	for(sentBytes = 0; 1; ){
		sentBytes += RingBuffer_InsertMult(&pRsrc->txRB, (void*)&BUF[sentBytes], len-sentBytes);
		uartTxPolling(pRsrc);
		if(RingBuffer_IsEmpty(&pRsrc->txRB))	break;	
	}
	return sentBytes;
}

static void uartTxSendString(UartRsrc_t *pRsrc, const char* FORMAT_ORG, ...){
	va_list ap;
	s16 bytes;
	char buff[512]={0};
	
	if(FORMAT_ORG == NULL)	return ;
	va_start(ap, FORMAT_ORG);
	bytes = vsnprintf(buff, 512, FORMAT_ORG, ap);
	va_end(ap);
	
	uartSend(pRsrc, (u8*)buff, bytes);
}

static void uartTxSendStringSync(UartRsrc_t *pRsrc, const char* FORMAT_ORG, ...){
	va_list ap;
	s16 bytes;
	char buff[512]={0};
	
	if(FORMAT_ORG == NULL)	return ;
	va_start(ap, FORMAT_ORG);
	bytes = vsnprintf(buff, 512, FORMAT_ORG, ap);
	va_end(ap);
	
	uartSendSync(pRsrc, (u8*)buff, bytes);
}
#endif

static u16 uartTxSendFrame(UartRsrc_t *pRsrc, const u8* BUF, u16 len){
	u16 crc;
	u8 buff[5];
	
	if(BUF == NULL || len==0)	return 0;

	crc = CRC16(BUF, len, 0xacca);
	buff[0] = UART_FRAM_HEAD&0xff;
	buff[1] = (UART_FRAM_HEAD>>8)&0xff;	
	buff[2] = (UART_FRAM_HEAD>>16)&0xff;
	uartSend(pRsrc, buff, 3);
	uartSend(pRsrc, BUF, len);
	buff[0] = crc & 0xff;
	buff[1] = (crc>>8) & 0xff;
	buff[2] = UART_FRAM_END&0xff;
	buff[3] = (UART_FRAM_END>>8)&0xff;
	buff[4] = (UART_FRAM_END>>16)&0xff;
	uartSend(pRsrc, buff, 5);

	return (len+8);
}

static u8 uartRxMonitor(UartRsrc_t *pRsrc){
	s16 bytesReceived;
	u8* pTmp, abandond;
	UART_HandleTypeDef *huart = pRsrc->huart;

  /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
	__HAL_UART_DISABLE_IT(huart, UART_IT_RXNE|UART_IT_PE);
	bytesReceived = huart->RxXferSize - huart->RxXferCount;
	if(bytesReceived > 0){
		//restart uart
		huart->pRxBuffPtr = pRsrc->rxNxtBuf;
		huart->RxXferCount = pRsrc->rxBufLen;
	}
	__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE|UART_IT_PE);

	if(bytesReceived <= 0)	return 0;
	
	// only keep the last received
	while(RingBuffer_GetFree(&pRsrc->rxRB) < bytesReceived){		
		RingBuffer_Pop(&pRsrc->rxRB, &abandond);
		pRsrc->errorCode |= BIT(0);
	}
	RingBuffer_InsertMult(&pRsrc->rxRB, pRsrc->rxCurBuf, bytesReceived);
	
	pTmp = pRsrc->rxCurBuf;
	pRsrc->rxCurBuf = pRsrc->rxNxtBuf;
	pRsrc->rxNxtBuf = pTmp;
	
	return (bytesReceived);
}

u16 fetchLineFromRingBuffer(RINGBUFF_T* rb, char* line, u16 len){
	u8 ret = 0;
	char *p = NULL;
	s32 i,lineLen=0, bytes, count;
		
	count = RingBuffer_GetCount(rb);
	if((count <= 0) || (line==NULL) || (len==0))	return 0;

	// only take the lase receive
	while(count > len){
		RingBuffer_Pop(rb, line);
		count = RingBuffer_GetCount(rb);
	}
	bytes = RingBuffer_PopMult(rb, line, len);
	RingBuffer_Flush(rb);

	// seek for end code
	for(i=0;i<bytes;i++){
		p = strstr(&line[i], CMD_END);	//be careful!! p can be out off buff, because 'line' may not end with '\0'
		if(p){
			// test overfloat memory
			if(p > (line+bytes-strlen(CMD_END))){
				p = NULL;
				break;
			}	
			lineLen = p-(char*)&line[i]+strlen(CMD_END);
			count = bytes - (i+lineLen);
			if(count > 0){
				RingBuffer_InsertMult(rb, &line[i+lineLen], count);
			}
			if(i>0){
				memmove(line, &line[i], (lineLen>len?len:lineLen));
				line[lineLen] = 0;
			}
			ret = 1;
			break;
		}
	}
	
	if(p==NULL){	RingBuffer_InsertMult(rb, line, bytes);	}

	return ret;
}


//better if len equ rxRB' pool len
static u16 uartRxFetchLine(UartRsrc_t *pRsrc, char* line, u16 len){
	return(fetchLineFromRingBuffer(&pRsrc->rxRB, line, len));
}

static u16 uartRxFetchFrame(UartRsrc_t *pRsrc, u8* frame, u16 frameLen){
	u16 i,j,crc0,crc1;
	u8 *head, *end, *pCrc, *buff = frame;
	u16 len = 0;
	s16 bytes,count;
	
	if(RingBuffer_GetCount(&pRsrc->rxRB) < (3+2+3))	return 0;	// 3(head) + 2(CRC) + 3(end)
		
	bytes = RingBuffer_PopMult(&pRsrc->rxRB, buff, frameLen);
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
	// if do not meet head, just keep last 2 bytes, this two may be head beginning
	if(head==NULL){
		RingBuffer_InsertMult(&pRsrc->rxRB, buff, bytes);
//		if(bytes > 2)	RingBuffer_InsertMult(&pRsrc->rxRB, &buff[bytes-2], 2);
//		else	RingBuffer_InsertMult(&pRsrc->rxRB, buff, bytes);
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
	// keep effective data
	if(end==NULL){
		//RingBuffer_InsertMult(&pRsrc->rxRB, head, bytes-(head-buff));
		RingBuffer_InsertMult(&pRsrc->rxRB, buff, bytes);
		return 0;
	}

	count = buff + bytes - (end+3);
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
		RingBuffer_Flush(&pRsrc->rxRB);
		return len;
	}

	return 0;
}

static s16 TjcSendCmd(UartRsrc_t* p, const char* FORMAT_ORG, ...){
	u8 buf[100] = {0};
	s32 len;
	va_list ap;
	s16 bytes;
	//take string
	if(FORMAT_ORG == NULL)	return -1;
	memset(buf, 0, 100);
	va_start(ap, FORMAT_ORG);
	bytes = vsnprintf((char*)buf, 100, FORMAT_ORG, ap);
	va_end(ap);

	len = strlen((char*)buf);
	if(len <= 0)	return -2;
	buf[len++] = 0xff;
	buf[len++] = 0xff;
	buf[len++] = 0xff;
	uartSend(p, buf, len);	// load to ringbuffer
//	p->uart.TxPolling(&p->uart.rsrc);	// send out
//	HAL_Delay(5);
	return bytes;
}

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
