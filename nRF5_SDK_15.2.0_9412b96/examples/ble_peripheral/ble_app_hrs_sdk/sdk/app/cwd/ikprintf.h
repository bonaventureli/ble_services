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
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "nrf_log.h"

typedef enum {
	IK_LOG_DEBUG =1,
	IK_LOG_INFO,
	IK_LOG_WARNING,
	IK_LOG_ERROR,
}IK_LOG_LEVEL;


#define ikLogPrintf(level ,...) { if(level == IK_LOG_DEBUG ) { NRF_LOG_DEBUG(__VA_ARGS__); } \
else if(level == IK_LOG_INFO ){NRF_LOG_INFO(__VA_ARGS__);} \
else if(level == IK_LOG_WARNING ) {NRF_LOG_WARNING(__VA_ARGS__);}	\
else if(level == IK_LOG_ERROR ) 	{NRF_LOG_ERROR(__VA_ARGS__);}	\
else{NRF_LOG_DEBUG(__VA_ARGS__); } }

#endif
