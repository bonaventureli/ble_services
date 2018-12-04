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


#include "ikcmdif.h"

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
   MslCarcmd(CARCMD_CMD_CENTRAL_CONTROLLOCK,CARCMD_PARAM_1);
  break;
  }
  case 2:{
   
  MslCarcmd(CARCMD_CMD_CENTRAL_CONTROLLOCK,CARCMD_PARAM_0);
  break;
  }
  case 3:{
   MslCarcmd(CARCMD_CMD_HATCHBACK_CONTROL,CARCMD_PARAM_1);
  break;
  }
	case 4:{
  MslCarcmd(CARCMD_CMD_ENGINE_CONTROL,CARCMD_PARAM_1);
  break;
  }
  case 7:{
   MslCarcmd(CARCMD_CMD_FIND_CAR,CARCMD_PARAM_1);
  break;
	}
	case 8:{
  MslCarcmd(CARCMD_CMD_ENGINE_CONTROL,CARCMD_PARAM_0);
  break;
  }
	case 11:{
  MslCarcmd(CARCMD_CMD_WINDOW_CONTROL,CARCMD_PARAM_1);
  break;
		}
	case 12:{
  MslCarcmd(CARCMD_CMD_WINDOW_CONTROL,CARCMD_PARAM_0);
  break;
		}
	case 13:{
		MslCarcmd(CARCMD_CMD_WINDOW_CONTROL,CARCMD_PARAM_1);
  break;
		}
	case 14:{
  MslCarcmd(CARCMD_CMD_WINDOW_CONTROL,CARCMD_PARAM_0);
  break;
		}
	case 28:{
  MslCarcmd(CARCMD_CMD_HATCHBACK_CONTROL,CARCMD_PARAM_0);
  break;
  }
		default: break;
}
}

void Status_Car(void)
{
// if (query_if_in_car()==1){
 MslCarcmd(CARCMD_CMD_POSION,CARCMD_PARAM_1);//in car   01 F1 00 01 01 
 NRF_LOG_INFO("out car.");
// }
// else if (query_if_in_car()==0){
 MslCarcmd(CARCMD_CMD_POSION,CARCMD_PARAM_2);//out car  01 F1 00 01 02 
 NRF_LOG_INFO("open engin.");
// }
}
void One_Key_Start(void)
{
 MslCarcmd(CARCMD_CMD_ENGINE_CONTROL,CARCMD_PARAM_1);//open engin    01 14 00 01 01 
}



BleNotify_CallBack gBleNotify;
DK_Cmd_Meg struct_cmd;
uint8_t *preply_data;
unsigned int outlen;
#define BLE_UUID_DIGITALKET_CMD_CHAR                     			 	 0xFFF5     /**< CMD characteristic UUID. */

void NotifyCallBack(BleNotify_CallBack bcb){
	gBleNotify = bcb;
}
void Receive_Task(const uint8_t * data)
{
 if((data[ee_Type]== MONITOR_TYPE_POSION)&&(data[ee_Cmd] == MONITOR_CMD_CAR_POSION)&&(data[ee_Length]==0x01)){
 switch(data[ee_Param]){
  case CARCMD_CMD_REQUSE:{ //03FF0001F0
   One_Key_Start();
  break;
  }
  case CARCMD_CMD_POSION:{  //03FF0001F1
   Status_Car();
  break;
  }
  default: break;
 }
} 
 else if(data[ee_Type]== MONITOR_TYPE){
	 switch(data[ee_Cmd]){
		 case MONITOR_CMD_CENTRAL_CONTROLLOCK:{
			 if(data[ee_Index]==0x00){
				 	 switch(data[ee_Param]){
							 case 0x00:{
								  struct_cmd.command = data[ee_Cmd];
									struct_cmd.index = data[ee_Index];
									struct_cmd.result = 0;
									ingeek_command_output_action(&struct_cmd,preply_data, &outlen);
									gBleNotify(BLE_UUID_DIGITALKET_CMD_CHAR, preply_data, outlen);
							 break;
							 }
							 case 0x01:{
									struct_cmd.command = data[ee_Cmd];
									struct_cmd.index = data[ee_Index];
									struct_cmd.result = 1;
									ingeek_command_output_action(&struct_cmd,preply_data, &outlen);
									gBleNotify(BLE_UUID_DIGITALKET_CMD_CHAR, preply_data, outlen);
							 break;
							 }
						 }
			 }
			 else if(data[ee_Index]==0x01){
				 	 switch(data[ee_Param]){
							 case 0x00:{
									struct_cmd.command = data[ee_Cmd];
									struct_cmd.index = data[ee_Index];
									struct_cmd.result = 0;
									ingeek_command_output_action(&struct_cmd,preply_data, &outlen);
									gBleNotify(BLE_UUID_DIGITALKET_CMD_CHAR, preply_data, outlen);
							 break;
							 }
							 case 0x01:{
									struct_cmd.command = data[ee_Cmd];
									struct_cmd.index = data[ee_Index];
									struct_cmd.result = 1;
									ingeek_command_output_action(&struct_cmd,preply_data, &outlen);
									gBleNotify(BLE_UUID_DIGITALKET_CMD_CHAR, preply_data, outlen);
							 break;
							 }
						 }
			 }
		 break;
		 }
		 case MONITOR_CMD_WINDOW_CONTROL:{
			 if(data[ee_Index]==0x00){
				 	 switch(data[ee_Param]){
							 case 0x00:{
								  struct_cmd.command = data[ee_Cmd];
									struct_cmd.index = data[ee_Index];
									struct_cmd.result = 0;
									ingeek_command_output_action(&struct_cmd,preply_data, &outlen);
									gBleNotify(BLE_UUID_DIGITALKET_CMD_CHAR, preply_data, outlen);
							 break;
							 }
							 case 0x01:{
									struct_cmd.command = data[ee_Cmd];
									struct_cmd.index = data[ee_Index];
									struct_cmd.result = 1;
									ingeek_command_output_action(&struct_cmd,preply_data, &outlen);
									gBleNotify(BLE_UUID_DIGITALKET_CMD_CHAR, preply_data, outlen);
							 break;
							 }
						 }
			 }
			 else if(data[ee_Index]==0x01){
				 	 switch(data[ee_Param]){
							 case 0x00:{
									struct_cmd.command = data[ee_Cmd];
									struct_cmd.index = data[ee_Index];
									struct_cmd.result = 0;
									ingeek_command_output_action(&struct_cmd,preply_data, &outlen);
									gBleNotify(BLE_UUID_DIGITALKET_CMD_CHAR, preply_data, outlen);
							 break;
							 }
							 case 0x01:{
									struct_cmd.command = data[ee_Cmd];
									struct_cmd.index = data[ee_Index];
									struct_cmd.result = 1;
									ingeek_command_output_action(&struct_cmd,preply_data, &outlen);
									gBleNotify(BLE_UUID_DIGITALKET_CMD_CHAR, preply_data, outlen);
							 break;
							 }
						 }
			 }
		 break;
		 }
		 case MONITOR_CMD_SKYLIGHT_CONTROL:{
			 if(data[ee_Index]==0x00){
				 	 switch(data[ee_Param]){
							 case 0x00:{
								  struct_cmd.command = data[ee_Cmd];
									struct_cmd.index = data[ee_Index];
									struct_cmd.result = 0;
									ingeek_command_output_action(&struct_cmd,preply_data, &outlen);
									gBleNotify(BLE_UUID_DIGITALKET_CMD_CHAR, preply_data, outlen);
							 break;
							 }
							 case 0x01:{
									struct_cmd.command = data[ee_Cmd];
									struct_cmd.index = data[ee_Index];
									struct_cmd.result = 1;
									ingeek_command_output_action(&struct_cmd,preply_data, &outlen);
									gBleNotify(BLE_UUID_DIGITALKET_CMD_CHAR, preply_data, outlen);
							 break;
							 }
						 }
			 }
			 else if(data[ee_Index]==0x01){
				 	 switch(data[ee_Param]){
							 case 0x00:{
									struct_cmd.command = data[ee_Cmd];
									struct_cmd.index = data[ee_Index];
									struct_cmd.result = 0;
									ingeek_command_output_action(&struct_cmd,preply_data, &outlen);
									gBleNotify(BLE_UUID_DIGITALKET_CMD_CHAR, preply_data, outlen);
							 break;
							 }
							 case 0x01:{
									struct_cmd.command = data[ee_Cmd];
									struct_cmd.index = data[ee_Index];
									struct_cmd.result = 1;
									ingeek_command_output_action(&struct_cmd,preply_data, &outlen);
									gBleNotify(BLE_UUID_DIGITALKET_CMD_CHAR, preply_data, outlen);
							 break;
							 }
						 }
			 }
		 break;
		 }
		 case MONITOR_CMD_HATCHBACK_CONTROL:{
			 if(data[ee_Index]==0x00){
				 	 switch(data[ee_Param]){
							 case 0x00:{
								  struct_cmd.command = data[ee_Cmd];
									struct_cmd.index = data[ee_Index];
									struct_cmd.result = 0;
									ingeek_command_output_action(&struct_cmd,preply_data, &outlen);
									gBleNotify(BLE_UUID_DIGITALKET_CMD_CHAR, preply_data, outlen);
							 break;
							 }
							 case 0x01:{
									struct_cmd.command = data[ee_Cmd];
									struct_cmd.index = data[ee_Index];
									struct_cmd.result = 1;
									ingeek_command_output_action(&struct_cmd,preply_data, &outlen);
									gBleNotify(BLE_UUID_DIGITALKET_CMD_CHAR, preply_data, outlen);
							 break;
							 }
						 }
			 }
			 else if(data[ee_Index]==0x01){
				 	 switch(data[ee_Param]){
							 case 0x00:{
									struct_cmd.command = data[ee_Cmd];
									struct_cmd.index = data[ee_Index];
									struct_cmd.result = 0;
									ingeek_command_output_action(&struct_cmd,preply_data, &outlen);
									gBleNotify(BLE_UUID_DIGITALKET_CMD_CHAR, preply_data, outlen);
							 break;
							 }
							 case 0x01:{
									struct_cmd.command = data[ee_Cmd];
									struct_cmd.index = data[ee_Index];
									struct_cmd.result = 1;
									ingeek_command_output_action(&struct_cmd,preply_data, &outlen);
									gBleNotify(BLE_UUID_DIGITALKET_CMD_CHAR, preply_data, outlen);
							 break;
							 }
						 }
			 }
		 break;
		 }
		 case MONITOR_CMD_ENGINE_CONTROL:{
			 if(data[ee_Index]==0x00){
				 	 switch(data[ee_Param]){
							 case 0x00:{
								  struct_cmd.command = data[ee_Cmd];
									struct_cmd.index = data[ee_Index];
									struct_cmd.result = 0;
									ingeek_command_output_action(&struct_cmd,preply_data, &outlen);
									gBleNotify(BLE_UUID_DIGITALKET_CMD_CHAR, preply_data, outlen);
							 break;
							 }
							 case 0x01:{
									struct_cmd.command = data[ee_Cmd];
									struct_cmd.index = data[ee_Index];
									struct_cmd.result = 1;
									ingeek_command_output_action(&struct_cmd,preply_data, &outlen);
									gBleNotify(BLE_UUID_DIGITALKET_CMD_CHAR, preply_data, outlen);
							 break;
							 }
						 }
			 }
			 else if(data[ee_Index]==0x01){
				 	 switch(data[ee_Param]){
							 case 0x00:{
									struct_cmd.command = data[ee_Cmd];
									struct_cmd.index = data[ee_Index];
									struct_cmd.result = 0;
									ingeek_command_output_action(&struct_cmd,preply_data, &outlen);
									gBleNotify(BLE_UUID_DIGITALKET_CMD_CHAR, preply_data, outlen);
							 break;
							 }
							 case 0x01:{
									struct_cmd.command = data[ee_Cmd];
									struct_cmd.index = data[ee_Index];
									struct_cmd.result = 1;
									ingeek_command_output_action(&struct_cmd,preply_data, &outlen);
									gBleNotify(BLE_UUID_DIGITALKET_CMD_CHAR, preply_data, outlen);
							 break;
							 }
						 }
			 }
		 break;
		 }
	 }

 }
 
}

/** 
 * @}
 */
