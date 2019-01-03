
/*********************************************************************
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 * @param   
 * @version 0.1
 * @return  None.
 * @auther lifei
 * @update time 2018.9.21
 */


 /*Header        Type 	Cmd 	Length    Data     End
 * 1Byte        1Byte   1Byte   1Byte     1Byte    1Byte
 * */
 
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>


#ifndef IKCMD_HANDLE_H
#define IKCMD_HANDLE_H

#define MONITOR_HEADER 0x7E
#define MONITOR_TYPE   0x02
#define MONITOR_TYPE_POSION   0x03

#define MONITOR_CMD_CENTRAL_CONTROLLOCK 0x01
#define MONITOR_CMD_WINDOW_CONTROL 0x02
#define MONITOR_CMD_SKYLIGHT_CONTROL 0x03
#define MONITOR_CMD_HATCHBACK_CONTROL 0x09
#define MONITOR_CMD_ENGINE_CONTROL 0x14
#define MONITOR_CMD_CAR_POSION 0xFF


#define MONITOR_CMD_CARMNTOR_MODE 0xFF

#define MONITOR_CMD_POSITION_MODE 0x01
#define MONITOR_CMD_STAND_MODE    0x02
#define MONITOR_CMD_END_CALIBRATION 0x03
#define MONITOR_CMD_ACCOUNT_REQUEST 0x04
#define MONITOR_CMD_WARN_INFORMATION 0x10
//...
#define MONITOR_CMD_HINT_INFORMATION 0X30



#define CARCMD_HEADER 0x7E
#define CARCMD_TYPE 0x01
#define CARCMD_CMD_CARBINDING 0x00
#define CARCMD_CMD_CENTRAL_CONTROLLOCK 0x01
#define CARCMD_CMD_WINDOW_CONTROL 0x02
#define CARCMD_CMD_SKYLIGHT_CONTROL 0x03
#define CARCMD_CMD_AIR_CONDITION 0x13
#define CARCMD_CMD_HATCHBACK_CONTROL 0x09
#define CARCMD_CMD_REQUSE 0xF0
#define CARCMD_CMD_POSION 0xF1
#define CARCMD_CMD_ENGINE_CONTROL 0x14
#define CARCMD_CMD_FIND_CAR 0x15
#define CARCMD_CMD_WINDSPEED 0x0A
#define CARCMD_END 0xFF

#define CARCMD_PARAM_0 0
#define CARCMD_PARAM_1 1
#define CARCMD_PARAM_2 2
#define CARCMD_PARAM_3 3
#define CARCMD_PARAM_4 4
#define CARCMD_PARAM_5 5
#define CARCMD_PARAM_6 6
#define CARCMD_PARAM_7 7


#define CARCMD_RESPONSE_HEADER 0x7E
#define CARCMD_RESPONSE_TYPE 0x02


#define offset_0 0
#define offset_1 1
#define offset_2 2
#define offset_3 3
#define offset_4 4
#define offset_5 5
#define offset_6 6
#define offset_7 7
#define offset_8 8
#define offset_9 9
#define offset_10 10
#define offset_11 11
#define offset_12 12
#define offset_13 13
#define offset_14 14
#define offset_15 15
#define offset_16 16
#define offset_17 17
#define offset_18 18
#define offset_19 19
#define offset_20 20
#define offset_21 21
#define offset_22 22

#define start_bit0 0
#define start_bit1 1
#define start_bit2 2
#define start_bit3 3
#define start_bit4 4
#define start_bit5 5
#define start_bit6 6
#define start_bit7 7

#define bit_len0 0
#define bit_len1 1
#define bit_len2 2
#define bit_len3 3
#define bit_len4 4
#define bit_len5 5
#define bit_len6 6
#define bit_len7 7
#define bit_len8 8

#define Byte_len1 1
#define Byte_len2 2
#define Byte_len4 4

typedef enum {
    e_Header,
    e_Type,
    e_Cmd,
    e_Length,//3
	  e_Data,
}e_Frame;
typedef enum {
    ee_Type,
    ee_Cmd,
		ee_Index,
    ee_Length,//3
	  ee_Param,
}ee_Frame;
typedef struct{
	uint8_t Header;
	uint8_t Type;
	uint8_t Cmd;
	uint8_t Length;
	uint8_t *Data;
	uint8_t End;
}T_Monitor_Data;

typedef struct{
	//uint8_t Header;
	uint8_t Type;
	uint8_t Cmd;
	uint8_t Index;
	uint8_t Length;
	uint8_t Param;
	//uint8_t End;
}T_CarCMD;

typedef struct{
	uint8_t Header;
	uint8_t Type;
	uint8_t Cmd;
	uint8_t Index;
	uint8_t Result;
	uint8_t End;
}T_CarCMD_Response;

typedef struct{
  uint8_t	 Data[22];
	uint8_t	 SentBusy;
	uint32_t SendTime;
	uint8_t  ReSentCount;
}T_CARDATA;
typedef struct{
	uint8_t	 Type;
	uint8_t	 Cmd;
	uint8_t Index;
	uint8_t  Length;
	uint8_t	 Data[57];
}T_MCARDATA;

typedef struct {
	uint8_t total_mileage[4];
	uint8_t continuout_mileage[2];
	uint8_t oil_mass;//6
	uint8_t low_oil;
	uint8_t voltage;//8
	
	uint8_t car_lamp:1;
	uint8_t brake_fluid:1;
	uint8_t driving_lockdoor:1;//9:2
	uint8_t passenger_lockdoor:1;
	uint8_t driving_door:1;
	uint8_t passenger_door:1;
	uint8_t rear_cover:1;
	uint8_t leftback_door:1;
	
	uint8_t rightback_door:1;
	uint8_t scuttle:4;
	uint8_t powerstate:2;
	uint8_t reserver1:1;
	
	uint8_t tirepressure_fl;//11
	uint8_t tirepressure_fr;
	uint8_t tirepressure_rl;
	uint8_t tirepressure_rr;//14
	uint8_t fetaltemperature_fl;
	uint8_t fetaltemperature_fr;
	uint8_t fetaltemperature_rl;
	uint8_t fetaltemperature_rr;
	
	uint8_t PM[2];//19 20
	uint8_t cloth_state:3;
	uint8_t reserver2:5;
	uint8_t average_oil_consumption;//22
	
}T_Car_Status;
#define NULL 0

extern T_CarCMD CarCMD;
void handle_monitor(uint8_t *Data, uint8_t frame_len);
uint8_t BllGetValueCarOneByte(T_CARDATA *RxData,uint8_t offset,uint8_t Bitstart,uint8_t Bitlen);
//void Carcmd(uint8_t cmd,uint8_t param);
uint8_t CardcmdResponse(T_CarCMD_Response * CarCMDResponse);


void MslCarcmd(uint8_t cmd,uint8_t param);

#endif //IKCMD_HANDLE_H
