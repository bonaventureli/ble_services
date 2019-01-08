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
#include "ble_hrs.h"
//#include "crc16.h"
BLE_HRS_DEF(m_hrs);                                                 /**< Heart rate service instance. */

extern void MslCarcmd(uint8_t cmd,uint8_t param);
extern uint32_t ble_send_notify(uint16_t uuid, uint8_t * data, uint16_t length);
//pfnWritePortCB_t   g_uartTxCB = 0;                                              /**< Heart rate service instance. */;d

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
		BleRequireMonitorData(100,5);
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

extern DK_Cmd_Meg gstruct_cmd;
uint8_t Cmdpreply_data[16];
unsigned int outlen;


void NotifyCallBack(BleNotify_CallBack bcb){
	gBleNotify = bcb;
}
void Receive_Task(const uint8_t * data)
{
int ret;
uint8_t status;
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
#define BLE_UUID_DIGITALKET_CMD_CHAR                     			 	 0xFFF5     /**< CMD characteristic UUID. */
  if(data[0] == 0x02){
	 switch(data[4]){
		 case 0x00:{
			 gstruct_cmd.result = 0x00;
		 break;
		 }
		 case 0x01:{
			 gstruct_cmd.result = 0x01;
		 break;
		 }
		 case 0x02:{
			 gstruct_cmd.result = 0x02;
		 break;
		 }
		 case 0x03:{
			 gstruct_cmd.result = 0x03;
		 break;
		 }
		 case 0x04:{
			 gstruct_cmd.result = 0x04;
		 break;
		 }
		 case 0x05:{
			 gstruct_cmd.result = 0x05;
		 break;
		 }
		 case 0x06:{
			 gstruct_cmd.result = 0x06;
		 break;
		 }
		 case 0x07:{
			 gstruct_cmd.result = 0x07;
		 break;
		 }
		 case 0x08:{
			 gstruct_cmd.result = 0x08;
		 break;
		 }
		 case 0x09:{
			 gstruct_cmd.result = 0x09;
		 break;
		 }
		 case 0x0A:{//10
			 gstruct_cmd.result = 0x0A;
		 break;
		 }
		 case 0x0B:{//12
			 gstruct_cmd.result = 0x0B;
		 break;
		 }
	 }
		 if((ret = ingeek_command_output_action(&gstruct_cmd,Cmdpreply_data, &outlen)) != INGEEK_OK || outlen != 16){
				NRF_LOG_INFO("error return ret = %d ",ret);
		}
				NRF_LOG_INFO("struct_cmd.result %x",gstruct_cmd.result);
				NRF_LOG_HEXDUMP_INFO(Cmdpreply_data, outlen);
				ble_send(&m_hrs, BLE_UUID_DIGITALKET_CMD_CHAR, Cmdpreply_data, outlen);
	}
HandleFrameData(data);
}
typedef enum {
    em_Type,
    em_Cmd,
		em_Index,
    em_Length,
	  em_Data,
}em_Frame;

#define MTYPE 0x03
#define MSTATUS 0xFE
#define MINDEX 0x00
#define MLENGTH 0x39
uint8_t Monitorpreply_data[MLENGTH];

void HandleFrameData(const uint8_t * data){
	 /*car monitor*/

if(data[em_Type] == MTYPE){
	if(data[em_Cmd] == MSTATUS){
		if(data[em_Index] == MINDEX){
			//if(data[em_Length] == MLENGTH){
				memcpy(Monitorpreply_data,data+4,0x39);
				NRF_LOG_HEXDUMP_INFO(Monitorpreply_data,sizeof(Monitorpreply_data));
				//ble_send(&m_hrs, BLE_UUID_DIGITALKET_CMD_CHAR, Monitorpreply_data, 0x39);
			//}

		}
	}
}
if(data[0]== 0xAA){
if(data[1]== 0xBB){
	switch(data[2]){
		case 0x01:{
			frontleftCarwindow();
		break;
		}
		case 0x02:{
			frontrightCarwindow();
		break;
		}
		case 0x03:{
			behindleftCarwindow();
		break;
		}
		case 0x04:{
			behindrightCarwindow();
		break;
		}
		case 0x05:{
			scuttle();
		break;
		}
		case 0x06:{
			frontleftdoor();
		break;
		}
		case 0x07:{
			frontrightdoor();
		break;
		}
		case 0x08:{
			behindleftdoor();
		break;
		}
		case 0x09:{
			behindrightdoor();
		break;
		}
		case 0x0A:{
			trunk();
		break;
		}
		case 0x0B:{
			motor();
		break;
		}
	}
}
}
}
typedef struct SignalPosition{
	 uint8_t  BitLength;
	 uint8_t  ByteOffset;
	 uint8_t  BitStart;
}T_MASK_MATRIX;  

const T_MASK_MATRIX MaskMatrix[] = {
/*NUM   BitLength  ByteOffset  BitStart*/
/* 0*/  {1,      		24,     1}, //frontleftCarwindow
/* 1*/  {1,     	 	24,     2}, //frontrightCarwindow
/* 2*/  {1,      		24,     3}, //behindleftCarwindow
/* 3*/  {1,      		24,     4}, //behindrightCarwindow
/* 4*/  {1,      		24,     5}, //scuttle
/* 5*/  {1,      		25,     3}, //frontleftdoor
/* 6*/  {1,      		25,     4}, //frontrightdoor
/* 7*/  {1,      		25,     5}, //behindleftdoor
/* 8*/  {1,      		25,     6}, //behindrightdoor
/* 9*/  {1,      		26,     0}, //trunk
/*10*/  {1,      		21,     5}, //motor
};
uint8_t *gRMoData;
uint8_t DataMask[] = {0x00,0x01,0x03,0x07,0x0F,0x1F,0x3F,0x7F,0xFF};


uint8_t *HandleMonitData(uint8_t *Data,uint8_t Select){
	//uint8_t *MonitorData(T_MCARDATA *RData,uint8_t Select){
	uint8_t Retval,Retval2,Retval3,Retval4,ByteStart,BitStart,BitLength,Scase;
	
	ByteStart = MaskMatrix[Select].ByteOffset;
	BitStart = MaskMatrix[Select].BitStart;
	
	Scase = MaskMatrix[Select].BitLength/8;
	if(Scase == 0){
		NRF_LOG_INFO("ByteStart %d",ByteStart);
		NRF_LOG_INFO("Data[ByteStart] %x",Data[ByteStart]);
		
		BitLength  = MaskMatrix[Select].BitLength;
		//Retval = (RData->Data[ByteStart]>>BitStart)&DataMask[BitLength];
		Retval = (Data[ByteStart]>>BitStart)&DataMask[BitLength];
		gRMoData = &Retval;
		//NRF_LOG_INFO("Retval %x",Retval);
	}
	else if (Scase == 1){
		//Retval = RData->Data[ByteStart];
		Retval = Data[ByteStart];
		gRMoData = &Retval;
	}
	else if(Scase == 2){
		Retval = Data[ByteStart];
//		Retval = RData->Data[ByteStart];
//		Retval2 = RData->Data[ByteStart+1];
		gRMoData = &Retval;
	}
	else if(Scase == 4){
		Retval = Data[ByteStart];
//		Retval = RData->Data[ByteStart];
//		Retval2 = RData->Data[ByteStart+1];
//		Retval3 = RData->Data[ByteStart+2];
//		Retval4 = RData->Data[ByteStart+3];
		gRMoData = &Retval;
	}
	return gRMoData;
}
extern uint8_t gUartRxDatazta[200];

#define FRONT_LEFT_CARWINDOW 0
#define FRONT_RIGHT_CARWINDOW 1
#define BEHIND_LEFT_CARWINDOW 2
#define BEHIND_RIGHT_CARWINDOW 3
#define DSCUTTLE 4
#define FRONT_LEFT_DOOR 5
#define FRONT_RIGHT_DOOR 6
#define BEHIND_LEFT_DOOR 7
#define BEHIND_RIGHT_DOOR 8
#define DTRUNK 9
#define DMOTOR 10

void TotalOdometer_km(){
	uint8_t data[4];
	HandleMonitData(Monitorpreply_data,5);
	memcpy(data,gRMoData,4);
}

void frontleftCarwindow(void){
	uint8_t data[1];
	HandleMonitData(Monitorpreply_data,FRONT_LEFT_CARWINDOW);
	memcpy(data,gRMoData,1);
	NRF_LOG_INFO("frontleftCarwindow %x",data[0]);
}

void frontrightCarwindow(void){
	uint8_t data[1];
	HandleMonitData(Monitorpreply_data,FRONT_RIGHT_CARWINDOW);
	memcpy(data,gRMoData,1);
	NRF_LOG_INFO("frontrightCarwindow %x",data[0]);
}

void behindleftCarwindow(void){
	uint8_t data[1];
	HandleMonitData(Monitorpreply_data,BEHIND_LEFT_CARWINDOW);
	memcpy(data,gRMoData,1);
	NRF_LOG_INFO("behindleftCarwindow %x",data[0]);
}

void behindrightCarwindow(void){
	uint8_t data[1];
	HandleMonitData(Monitorpreply_data,BEHIND_RIGHT_CARWINDOW);
	memcpy(data,gRMoData,1);
	NRF_LOG_INFO("behindrightCarwindow %x",data[0]);
}

void scuttle(void){
	uint8_t data[1];
	HandleMonitData(Monitorpreply_data,DSCUTTLE);
	memcpy(data,gRMoData,1);
	NRF_LOG_INFO("scuttle %x",data[0]);
}

void frontleftdoor(void){
	uint8_t data[1];
	HandleMonitData(Monitorpreply_data,FRONT_LEFT_DOOR);
	memcpy(data,gRMoData,1);
	NRF_LOG_INFO("frontleftdoor %x",data[0]);
}

void frontrightdoor(void){
	uint8_t data[1];
	HandleMonitData(Monitorpreply_data,FRONT_RIGHT_DOOR);
	memcpy(data,gRMoData,1);
	NRF_LOG_INFO("frontrightdoor %x",data[0]);
}
void behindleftdoor(void){
	uint8_t data[1];
	HandleMonitData(Monitorpreply_data,BEHIND_LEFT_DOOR);
	memcpy(data,gRMoData,1);
	NRF_LOG_INFO("behindleftdoor %x",data[0]);
}
void behindrightdoor(void){
	uint8_t data[1];
	HandleMonitData(Monitorpreply_data,BEHIND_RIGHT_DOOR);
	memcpy(data,gRMoData,1);
	NRF_LOG_INFO("behindrightdoor %x",data[0]);
}

void trunk(void){
	uint8_t data[1];
	HandleMonitData(Monitorpreply_data,DTRUNK);
	memcpy(data,gRMoData,1);
	NRF_LOG_INFO("trunk %x",data[0]);
}
void motor(void){
	uint8_t data[1];
	data[0] = *HandleMonitData(Monitorpreply_data,DMOTOR);
	NRF_LOG_INFO("motor %x",data[0]);
}


typedef struct{
	uint8_t Type;
	uint8_t Cmd;
	uint8_t Index;
	uint8_t Length;
	uint8_t Counter;
	uint8_t IntervalTime;
}T_MonitorData;

#define CARMOTOR_TYPE 0x07
#define CARREQUIRE_MONITOR 0x01

void SendMonitorDataQ(uint8_t cmd,uint8_t counter,uint8_t interval_time)
{
	T_MonitorData MonitorData;
	MonitorData.Type = CARMOTOR_TYPE;
	MonitorData.Cmd = cmd;
	MonitorData.Index = 0x00;
	MonitorData.Length = 0x02;
	MonitorData.Counter = counter;
	MonitorData.IntervalTime = interval_time;
	NRF_LOG_HEXDUMP_INFO((uint8_t*)&MonitorData,sizeof(T_MonitorData));
}

void BleRequireMonitorData(uint8_t counter,uint8_t interval_time){
	SendMonitorDataQ(CARREQUIRE_MONITOR,100,5);
}

/** 
 * @}
 */
