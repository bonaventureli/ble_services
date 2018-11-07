/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "ikcmdif.h"
#include "ikcmdhandle.h"

extern void MslCarcmd(uint8_t cmd,uint8_t param);

//pfnWritePortCB_t   g_uartTxCB = 0;


//int ikcmdUartSend(uint8_t *inBuf ,uint32_t size)
//{
//    if (g_uartTxCB)
//        g_uartTxCB(inBuf,size,0);
//   return 1;
//}

//int ikcmdInitialize(pfnWritePortCB_t inUartTx)
//{
//    g_uartTxCB = inUartTx;
//	  return 0;
//}

//dkRetCode_t ikcmdUartPortRxNotify(const uint8_t * inBuf,uint32_t inSize)
//{
//   MslPeriodTask(inBuf,inSize);
//   return DK_PACK_SUCCESS;
//}

int  ikcmdStart(void)
{
	
}


void ikcmdSendUart(uint8_t cmd)
{
 switch (cmd){
  case 0:{
   MslCarcmd(CARCMD_CMD_HATCHBACK_CONTROL,CARCMD_PARAM_0);
  break;
  }
  case 1:{
   MslCarcmd(CARCMD_CMD_CENTRAL_CONTROLLOCK,CARCMD_PARAM_1);//open door
  break;
  }
  case 2:{
   
  MslCarcmd(CARCMD_CMD_CENTRAL_CONTROLLOCK,CARCMD_PARAM_0);//close door
  break;
  }
  case 3:{
   MslCarcmd(CARCMD_CMD_HATCHBACK_CONTROL,CARCMD_PARAM_1);//open batch back
  break;
  }
  case 7:{
   MslCarcmd(CARCMD_CMD_FIND_CAR,CARCMD_PARAM_1);//find car
		
  break;
  }
	case 100:{
		MslCarcmd(CARCMD_CMD_REQUSE,CARCMD_PARAM_0);//requse over
		MslCarcmd(CARCMD_CMD_REQUSE,CARCMD_PARAM_1);//requse once
		MslCarcmd(CARCMD_CMD_REQUSE,CARCMD_PARAM_2);//requse series
		
		MslCarcmd(CARCMD_CMD_ENGINE_CONTROL,CARCMD_PARAM_0);//close engin
		MslCarcmd(CARCMD_CMD_ENGINE_CONTROL,CARCMD_PARAM_1);//open engin
		
	 MslCarcmd(CARCMD_CMD_POSION,CARCMD_PARAM_0);//no phone
	 MslCarcmd(CARCMD_CMD_POSION,CARCMD_PARAM_1);//in car
	 MslCarcmd(CARCMD_CMD_POSION,CARCMD_PARAM_2);//out car
	}
  default: break;
 }
}

void Status_Car(void)
{
// if (query_if_in_car()==1){
// MslCarcmd(CARCMD_CMD_POSION,CARCMD_PARAM_1);//in car
// }
// else if (query_if_in_car()==0){
// MslCarcmd(CARCMD_CMD_POSION,CARCMD_PARAM_2);//out car
// }
}
void One_Key_Start(void)
{
 MslCarcmd(CARCMD_CMD_ENGINE_CONTROL,CARCMD_PARAM_1);//open engin
}
void Receive_Task(uint8_t * data)
{
 //03FF01F0
 //03FF01F1
 if((data[0]==0x03)&&(data[1]==0xFF)&&(data[2]==0x01)){
 switch(data[3]){
  case CARCMD_CMD_REQUSE:{//0xF0
   One_Key_Start();
  break;
  }
  case CARCMD_CMD_POSION:{//0xF1
   Status_Car();
  break;
  }
  default: break;
 }
} 
}

/** 
 * @}
 */
