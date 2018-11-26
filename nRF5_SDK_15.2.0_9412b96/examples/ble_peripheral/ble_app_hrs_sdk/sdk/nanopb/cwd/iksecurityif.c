#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "ikcmdif.h"
#include "digital_key_api.h"
#include "iksecurityif.h"
#include "ikble_hrs.h"

#include "iktask.h"
#include "ikprintf.h"

#define SESSIONMAXBIT 112
#define INFOCHARMAXBIT 29
#define DK_REPLAY_MAXBIT 128

extern uint32_t ble_send_notify(uint16_t uuid, uint8_t * data, uint16_t length);
extern uint32_t ikble_set_se_info(uint8_t * data, uint16_t length);

bool ikSecurityDKHandleCharacter(uint16_t secChar, uint8_t *inCharValue,uint32_t inLen)
{
    bool err_code = false;
    uint8_t status = 0;
    ikLogPrintf(IK_LOG_WARNING,"ikCharacter %x", secChar);

    switch(secChar)
    {
    case BLE_DIGITAKKEY_EVT_INFO:
    {
			
        unsigned int outlen;
        uint8_t preply_data[DK_REPLAY_MAXBIT] = {0};
				
        if(ingeek_push_info((unsigned char *)inCharValue, inLen) != INGEEK_OK)
            break;

        status = ingeek_get_sec_status();
        if(ingeek_pull_info(preply_data, &outlen) != INGEEK_OK )
            break;

        status = ingeek_get_sec_status();
        ikble_set_se_info(preply_data, outlen);
        ikble_set_se_status(0);
        ikLogPrintf(IK_LOG_INFO,"write info length =%d", outlen);
        err_code = true;
        ik_flash_timers_start(ingeek_write_flash);
			
        break;
    }
    case BLE_DIGITAKKEY_EVT_AUTH:
    {
        int ret = 0;
        ikLogPrintf(IK_LOG_INFO,"auth event into ",1);
        if( (ret = ingeek_push_auth((unsigned char *)inCharValue, inLen, 0, 0)) != INGEEK_OK){
            ikLogPrintf(IK_LOG_WARNING,"ingeek_push_auth error %d",ret);
            break;
        }
        status = ingeek_get_sec_status();
        ble_send_notify(BLE_UUID_DIGITALKET_STATUS_CHAR, &status, 1);
        ikble_set_se_status(status);
        err_code = true;
        ikLogPrintf(IK_LOG_INFO,"ik auth success");
        break;
    }
    case BLE_DIGITAKKEY_EVT_SESSION:
    {
        unsigned int outlen;
        uint8_t preply_data[DK_REPLAY_MAXBIT] = {0};
        int ret = 0;
        ikLogPrintf(IK_LOG_INFO,"session event ");

        if((ret = ingeek_push_session((unsigned char *)inCharValue, inLen, preply_data, &outlen))!=0){
           ikLogPrintf(IK_LOG_WARNING,"ingeek_push_session fail error=%d ",ret);
           break;
        }
        status = ingeek_get_sec_status();
        //NRF_LOG_INFO("ingeek_push_session status=%d ",status);
        //NRF_LOG_HEXDUMP_INFO((uint8_t *)preply_data, (uint16_t)outlen);
        ble_send_notify(BLE_UUID_DIGITALKET_SESSION_CHAR, preply_data, outlen);
        ble_send_notify(BLE_UUID_DIGITALKET_STATUS_CHAR, &status, 1);
        ikble_set_se_status(status);
        ikLogPrintf(IK_LOG_INFO,"session end notify data len = %d ",outlen);
        err_code = true;
        break;
    }
    case BLE_DIGITAKKEY_EVT_CMD:
    {
        uint8_t preply_data[50];
        unsigned int outlen;
        DK_Cmd_Meg struct_cmd;
        uint8_t cmd =0;
        int ret = 0;
			
        if((ret = ingeek_command_input_action(inCharValue, inLen, &struct_cmd)) != 0x0){
           ikLogPrintf(IK_LOG_WARNING,"ingeek_command_input_action error is %d  ",ret);
           break;         
        }

        cmd = (uint8_t)(struct_cmd.command);
        ikcmdSendUart(cmd);
        if((ret = ingeek_command_output_action(&struct_cmd,preply_data, &outlen)) != INGEEK_OK \
					|| outlen != 16){
           ikLogPrintf(IK_LOG_WARNING,"ingeek_command_output_action error is %d ",ret);
           break;
        }
     
        ikLogPrintf(IK_LOG_WARNING,"cmd =%d ",cmd);
        ble_send_notify(BLE_UUID_DIGITALKET_CMD_CHAR, preply_data, outlen);
        err_code = true;
        break;
    }
#if 0
    case BLE_UUID_DIGITALKET_RSSI_CHAR:
        break;

    case BLE_UUID_DIGITALKET_VERSION_CHAR:
        break;

    case BLE_UUID_DIGITALKET_STATUS_CHAR:
        break;
#endif
    default:
        // should not reach here!
        break;
    }

    return err_code;
}

bool ikSecurityIsAuthorized()
{
   return ingeek_get_sec_status() == WRITE_SESSION;
}

bool ikSetPSCalibration(const unsigned char * bleMac,unsigned char psvalue,unsigned char pevalue){
	 ingeek_SetPSCalibration(bleMac,psvalue,pevalue);
	 ik_flash_timers_start(ingeek_SaveCalibration);
}

bool ikGetPSCalibration(const unsigned char * bleMac,unsigned char *psvalue,unsigned char *pevalue){
	return ingeek_GetPSCalibration(bleMac,psvalue,pevalue);
}


bool ikSecuritySetRunningStatus(uint16_t bleStatus)
{
    switch(bleStatus)
    {
    case DK_SYS_INIT:
        ingeek_se_init();
        break;
    case DK_BLE_INIT:
        if(ingeek_get_sec_status() == 0)
        {
           uint8_t preply_data[DK_REPLAY_MAXBIT] = {0};
           unsigned int outlen = 0;
					 unsigned char ps = 0,pe =0;
           if(ingeek_pull_info(preply_data, &outlen) != INGEEK_OK )
              break;
					 
           ikble_set_se_info(preply_data, outlen);
           ikble_set_se_status(0);
					 // 
					  
					 //ikGetPSCalibration("123456",&ps,&pe);
					 //NRF_LOG_INFO("init get ps pe =%d %d ",ps,pe);

        }
        break;
    case DK_BLE_ADV_FAST:      //Advertising event.
        break;
    case DK_BLE_ADV_IDLE:      //Advertising event.
        break;
    case DK_BLE_CONNECTED:
        break;
    case DK_BLE_DISCONNECTED:
      if(ingeek_se_final() != CARINFO_INVALID)
         ikble_set_se_status(0);
        break;
    case DK_BLE_GATTS_TIMEOUT:
    case DK_BLE_GATTC_TIMEOUT:
        //ingeek_se_init();
        //ikble_set_se_status((ingeek_get_sec_status()== 0)? 1: 0xff);
        break;
    default:
        break;
    }

    return true;
}


/*********************************************************************
*********************************************************************/
