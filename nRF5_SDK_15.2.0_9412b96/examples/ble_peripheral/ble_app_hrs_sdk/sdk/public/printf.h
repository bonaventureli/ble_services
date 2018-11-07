/**
  ******************************************************************************
  * @file    printf.h
  * @author  bPanda Team
  * @version V1.0.0
  * @date    13-March-2018
  * @brief   
  ******************************************************************************
  * @attention
  *         
  ******************************************************************************
  */


#ifndef __PRINTF_H
#define __PRINTF_H

#include "queue.h"

#define PRINTF_BUFF_SIZE    sizeof(u8_t)
#define PRINTF_BUFF_CNT     1024
#define PRINTF_MAX_CNT      512

#define DEBUG_UART          UART2
#define DEBUG_RX_PORT       PORT10
#define DEBUG_RX_PIN        GPIO_Pin_13
#define DEBUG_TX_PORT       PORT10
#define DEBUG_TX_PIN        GPIO_Pin_14
#define DEBUG_BAUDRATE      115200

#define dbg_printf          DEBUG_PRINTF

#ifdef  DEBUG
#define DEBUG_PRINTF(format)    (DebugPrintf ## format)
#else
#define DEBUG_PRINTF(format)    
#endif

void DebugPrintfInit(void);
void DebugPrintf(const u8_t *format, ...);
void assert_failed(const s8_t* file, u32_t line);


#endif
