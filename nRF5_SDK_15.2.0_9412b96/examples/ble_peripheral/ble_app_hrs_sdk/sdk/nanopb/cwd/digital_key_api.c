/************************************************************************************
 *************************************************************************************
 * Include
 *************************************************************************************
 ************************************************************************************/

#include "DigitalKey.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "cmac.h"
#include "cipher_export.h"
#include "md_export.h"
#include <string.h>
#include "digital_key_api.h"
#include "ikprintf.h"

/************************************************************************************
 *************************************************************************************
 * Private macros
 *************************************************************************************
 ************************************************************************************/
#define  VIN_LEN    20
#define  LENGTH_8	8
#define  LENGTH_16	16
#define  LENGTH_32	32
#define  LENGTH_48  	48
#define  MAX_BUFF_LEN   256

#define VIN_OFFSET  0
#define KEY_OFFSET  64
#define SEID_OFFSET 112
#define PSCAL_OFFSET 130


/************************************************************************************
 *************************************************************************************
 * Private type definitions
 *************************************************************************************
 ************************************************************************************/
typedef struct{
    unsigned char CMPK[LENGTH_32];
    unsigned char VCK[LENGTH_16];
    unsigned char SK[LENGTH_16];
    unsigned char SSC[LENGTH_16];
}CRYPTO_T;


typedef struct{
    //unsigned int  KPRE;     //comment out by yx on 6/27
    int KPRE;
    unsigned char TRnd[LENGTH_8];
    unsigned char KIFD[LENGTH_32];
    unsigned char SEID[LENGTH_16];
    unsigned char Info_md[LENGTH_32];
    unsigned char s_cmac[LENGTH_16];
}AUTH_G;


typedef struct{
    unsigned char   status;
    unsigned int    end_time;
    unsigned char   vin[VIN_LEN];
    CRYPTO_T  	    Keyinfo;
    AUTH_G          auth_g;
}Ingeek_StateM_t;



/************************************************************************************
 *************************************************************************************
 * Private memory declarations
 *************************************************************************************
 ************************************************************************************/
static unsigned char* version="dk-v1.0";
static read_CallBack g_readcb = NULL;
static write_CallBack g_writecb = NULL;
static Rand_CallBack g_randcb = NULL;

#ifdef IK_DYNA_MALLOC
static unsigned char* uTempBuff = 0;
#else
static unsigned char uTempBuff[MAX_BUFF_LEN]={0};
#endif

/*
 save 3 bt mac with ps calibration. mac is saved at 0 to 5 bit 
*/
#define CAL_NUMBER  3
#define BT_MAC_LEN  6
#define BT_PSPE_LEN 2

static uint8_t s_psCalibration[CAL_NUMBER* (BT_MAC_LEN + BT_PSPE_LEN)] = {0};


static unsigned char * sTempFlashkey = 0; //save to flash
static Ingeek_StateM_t statem;
static ingeek_DK_SessionG session_message_g=ingeek_DK_SessionG_init_zero;
static unsigned char dkseid[LENGTH_16]={0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
                                        0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01};

#if 0
unsigned char* whitelist[16] ={0};
int record_num = 0;
typedef unsigned int UTCTime;
#endif

/************************************************************************************
 *************************************************************************************
 * Private functions prototypes
 *************************************************************************************
 ************************************************************************************/
static void set_sec_status(Sec_status status);

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/
/*
static void hex_dump(unsigned char* name, unsigned char* buffer, int len)
{
        int uIdx=0;

        DebugPrintf("%s: ",name);
        for (uIdx = 0; uIdx < len; uIdx++)
        {
                DebugPrintf("%02X ", buffer[uIdx]);
        }
        DebugPrintf("\n");
}*/


static int PBEncode_local_data(void* buffer, unsigned int length, void*pb_struct, const pb_field_t fields[])
{
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, length);
    if(! pb_encode(&stream, fields, pb_struct))
    {
        ikLogPrintf(IK_LOG_ERROR,"pbEncode %s\n", PB_GET_ERROR(&stream));
        return INGEEK_FAILED;
    }

    return (stream.bytes_written);
}


static int PBDecode_client_data(void* buffer, unsigned int length, void*pb_struct, const pb_field_t fields[])
{
    pb_istream_t stream = pb_istream_from_buffer(buffer, length);
    if(!(pb_decode(&stream, fields, pb_struct)))
    {
        ikLogPrintf(IK_LOG_ERROR,"pbDecode %s\n", PB_GET_ERROR(&stream));
        return INGEEK_FAILED;
    }

    return INGEEK_OK;
}

static int Handle_write_Info(ingeek_DK_InfoW* info_message)
{
    int ret=0;
    unsigned int keylen=0;
    memset(statem.vin,0,VIN_LEN);
    memcpy(statem.vin,info_message->VIN,VIN_LEN);
#if 0
    if(!g_writecb && g_writecb((unsigned char*)info_message->VIN, VIN_LEN, 0) != 0)
        return INGEEK_FAILED_WRITECB;

    if(g_writecb(info_message->KEY.bytes, info_message->KEY.size, 64) != 0)
        return INGEEK_FAILED_WRITECB;
#else
	if(!sTempFlashkey) sTempFlashkey = (unsigned char*)malloc(LENGTH_48);
    memcpy(sTempFlashkey,info_message->KEY.bytes,info_message->KEY.size);
#endif
    memset(statem.Keyinfo.SSC,0,LENGTH_16);
    ret=ingeek_cipher_aes_cbc((unsigned char *)info_message->VIN,statem.Keyinfo.SSC,info_message->KEY.bytes,
                              info_message->KEY.size,statem.Keyinfo.CMPK,&keylen,INGEEK_DECRYPT);
    if(ret != 0 || keylen != LENGTH_32)
        return INGEEK_FAILED_DECRYPT;

    return INGEEK_OK;
}

int ingeek_write_flash()
{
     if( g_writecb((unsigned char*)statem.vin, VIN_LEN, VIN_OFFSET) != 0 \
         || g_writecb((unsigned char*)sTempFlashkey, LENGTH_48, KEY_OFFSET) !=0 )
      return INGEEK_FAILED_WRITECB ;

      return 0;
}

static int Handle_write_data(unsigned char *input,unsigned int ilen)
{
    ingeek_DK_InfoW info_message=ingeek_DK_InfoW_init_zero;
    if(PBDecode_client_data(input,ilen,&info_message,ingeek_DK_InfoW_fields) != INGEEK_OK \
       || info_message.KEY.size != LENGTH_48 )
        return INGEEK_FAILED_PBDECODE;

    return Handle_write_Info(&info_message);
}


static int Calculate_VCK(void)
{
    const ingeek_md_info_t *md_info=NULL;
    if( NULL == (md_info = Get_md_info_from_type(INGEEK_MD_SHA256)))
        return INGEEK_FAILED_MDINFO;

    memset(statem.Keyinfo.VCK,0, LENGTH_16);
    if(0 != ingeek_message_digest_hmac(md_info,statem.Keyinfo.CMPK,LENGTH_32,statem.auth_g.Info_md,LENGTH_16,statem.Keyinfo.VCK))
        return INGEEK_FAILED_HMAC;

    return INGEEK_OK;
}


static int Calculate_SK(void *kicc, int klen)
{
    unsigned char mdout[LENGTH_32]={0};
    unsigned char tail[4]={0x00,0x00,0x00,0x01};
    const ingeek_md_info_t *md_info=NULL;

    /*get rand*/
    if(INGEEK_OK != g_randcb(NULL,statem.auth_g.KIFD, LENGTH_32))
        return INGEEK_FAILED_RANDCB;

    if( (md_info = Get_md_info_from_type(INGEEK_MD_SHA256)) == NULL )
        return INGEEK_FAILED_MDINFO;

    memset(uTempBuff,0,MAX_BUFF_LEN);
    memcpy(uTempBuff,statem.auth_g.KIFD,LENGTH_32);
    memcpy(uTempBuff + LENGTH_32,kicc,klen);
    memcpy(uTempBuff + LENGTH_32 + klen,tail,sizeof(tail));

    if(0 != ingeek_message_digest(md_info,uTempBuff,LENGTH_32 + klen + 4,mdout))
        return INGEEK_FAILED_MD;

    /*SK:KIFD||KICC||00 00 00 01*/
    memcpy(statem.Keyinfo.SK,mdout,LENGTH_16);
    return INGEEK_OK;
}


static int Verify_session_G(ingeek_DK_SessionG *session_message_g)
{
    if(0 != (memcmp(session_message_g->SEID.bytes,statem.auth_g.SEID,session_message_g->SEID.size)))
        return INGEEK_FAILED_SEID;

    if(0 != (memcmp(session_message_g->TRnd.bytes,statem.auth_g.TRnd,session_message_g->TRnd.size)))
        return INGEEK_FAILED_TRND;

    //must fixed cwd
    /*
    if(session_message_g->ENDTIME.seconds > statem.endtime){
        return -3;
    }
    */

    return INGEEK_OK;
}


static int Handle_Character_Session_G(ingeek_DK_SessionG *session_message_g,unsigned char *input,unsigned int ilen)
{
    int ret=0;
    unsigned int outLen = 0;

    if( (ret = Calculate_VCK()) != INGEEK_OK )
        return ret;

    memset(statem.Keyinfo.SSC,0x00,LENGTH_16);
    memset(uTempBuff,0x00,MAX_BUFF_LEN);
    ret=ingeek_cipher_aes_cbc(statem.Keyinfo.VCK,statem.Keyinfo.SSC,input,ilen,uTempBuff,&outLen,INGEEK_DECRYPT);
    if(ret != INGEEK_OK)
        return INGEEK_FAILED_DECRYPT;

    /*PlaintextLen  remove padding len*/
    ret=ingeek_AesCmacVerify(statem.auth_g.s_cmac,LENGTH_16,uTempBuff,outLen,statem.Keyinfo.VCK, LENGTH_16);
    if(ret != 0)
        return INGEEK_FAILED_CMAC;

    ret=PBDecode_client_data(uTempBuff,outLen,session_message_g,ingeek_DK_SessionG_fields);

    if(ret != INGEEK_OK ||session_message_g->CRnD.size != LENGTH_8 ||session_message_g->SEID.size != LENGTH_16)
        return INGEEK_FAILED_PBDECODE;

    ret=Verify_session_G(session_message_g);
    if(ret != INGEEK_OK)
        return ret;

    statem.auth_g.KPRE=session_message_g->KPRE;
    return INGEEK_OK;
}


static int combine_data_SS(ingeek_DK_SessionG *session_message, void* output, unsigned int* olen)
{
    ingeek_DK_SessionG message_g = ingeek_DK_SessionG_init_zero;
    message_g.avno=session_message->avno;
    message_g.TRnd.size = LENGTH_8;
    memcpy(message_g.TRnd.bytes,statem.auth_g.TRnd,LENGTH_8);

    message_g.CRnD.size = LENGTH_8;
    memcpy(message_g.CRnD.bytes,session_message->CRnD.bytes,LENGTH_8);

    message_g.SEID.size = LENGTH_16;
    memcpy(message_g.SEID.bytes,statem.auth_g.SEID,LENGTH_16);

    message_g.TEEID.size = LENGTH_16;
    memcpy(message_g.TEEID.bytes,session_message->TEEID.bytes,LENGTH_16);

    message_g.has_KIFD=true;
    message_g.KIFD.size = LENGTH_32;
    memcpy(message_g.KIFD.bytes,statem.auth_g.KIFD,LENGTH_32);

    message_g.has_KPRE=true;
    message_g.KPRE=session_message->KPRE;
    message_g.has_ENDTIME=true;
    message_g.ENDTIME.seconds=session_message->has_ENDTIME;
    unsigned int tempLen = 0;
    if( (tempLen = PBEncode_local_data(uTempBuff,MAX_BUFF_LEN,&message_g,ingeek_DK_SessionG_fields)) == INGEEK_FAILED)
        return INGEEK_FAILED_PBENCODE;

    memset(statem.Keyinfo.SSC,0x00,LENGTH_16);

    if(ingeek_cipher_aes_cbc(statem.Keyinfo.VCK,statem.Keyinfo.SSC,uTempBuff,tempLen,output,olen,INGEEK_ENCRYPT) != INGEEK_OK)
        return INGEEK_FAILED_ENCRYPT;

    return INGEEK_OK;
}


/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

int ingeek_pull_info(unsigned char *output, unsigned int* olen)
{
    ingeek_DK_InfoR info_message=ingeek_DK_InfoR_init_default;
    if(output == NULL || olen == NULL)
        return INGEEK_FAILED_PARAM;

    if(ingeek_get_sec_status() != CARINFO_VALID)
        return INGEEK_FAILED_STATUS;

    info_message.TRnd.size = LENGTH_8;
    /*get TRnd*/
    if(INGEEK_OK != g_randcb(NULL,info_message.TRnd.bytes, LENGTH_8))
        return INGEEK_FAILED_RANDCB;

    memcpy(info_message.VIN,statem.vin,VIN_LEN);
    if((*olen = PBEncode_local_data(output,MAX_BUFF_LEN,&info_message,ingeek_DK_InfoR_fields)) == INGEEK_FAILED)
        return INGEEK_FAILED_PBENCODE;

    memcpy(statem.auth_g.TRnd,info_message.TRnd.bytes,LENGTH_8);
    set_sec_status(READ_INFO);

    return INGEEK_OK;
}


int ingeek_push_info(unsigned char *input, int ilen)
{
    int ret=0;
    if(input == NULL || ilen <= 0 || g_writecb == NULL)
        return INGEEK_FAILED_PARAM;

    ret = Handle_write_data(input, ilen);
    if(ret == INGEEK_OK)
    {
        set_sec_status(CARINFO_VALID);
    }
    else
    {
        memset(statem.vin,0x00,sizeof(statem.vin));//clear vin
        memset(statem.Keyinfo.CMPK,0x00,LENGTH_32);//clear CMPK
        set_sec_status(CARINFO_INVALID);
    }

    return ret;
}

int ingeek_push_auth(unsigned char *input, unsigned int ilen, unsigned char* output, unsigned int *olen)
{
    ingeek_DK_AuthG auth_message_g=ingeek_DK_AuthG_init_zero;

    if(input == NULL || ilen <= 0 /*|| output == NULL || olen == NULL*/)
    {
        set_sec_status(CARINFO_VALID);
        return INGEEK_FAILED_PARAM;
    }

    if(ingeek_get_sec_status() != READ_INFO)
    {
        set_sec_status(CARINFO_VALID);
        return INGEEK_FAILED_STATUS;
    }

    if(PBDecode_client_data(input,ilen,&auth_message_g,ingeek_DK_AuthG_fields) != INGEEK_OK \
            || auth_message_g.df.size != LENGTH_32 || auth_message_g.s_cmac.size != LENGTH_16)
    {
        set_sec_status(CARINFO_VALID);
        memset(statem.Keyinfo.SK, 0x00, LENGTH_16);//clear SK
        return INGEEK_FAILED_PBDECODE;
    }

    memcpy(statem.auth_g.Info_md,auth_message_g.df.bytes,auth_message_g.df.size);
    memcpy(statem.auth_g.s_cmac,auth_message_g.s_cmac.bytes,auth_message_g.s_cmac.size);
    ingeek_set_seid(dkseid);
    set_sec_status(WRITE_AUTH);

    return INGEEK_OK;

}


int ingeek_push_session(unsigned char *input,unsigned int ilen,unsigned char* output, unsigned int* olen)
{
    int ret = 0;
    if(input == NULL || ilen <= 0 || output == NULL || olen == NULL)
    {
        set_sec_status(CARINFO_VALID);
        return INGEEK_FAILED_PARAM;
    }

    if(ingeek_get_sec_status() != WRITE_AUTH)
    {
        set_sec_status(CARINFO_VALID);
        return INGEEK_FAILED_STATUS;
    }

    memset(&session_message_g,0, sizeof(ingeek_DK_SessionG));  //comment out by yx
    ret=Handle_Character_Session_G(&session_message_g, input, ilen);
    if(ret != INGEEK_OK)
    {
        set_sec_status(CARINFO_VALID);
        return ret;
    }

    ret=Calculate_SK(session_message_g.KICC.bytes,session_message_g.KICC.size);
    if(ret != INGEEK_OK)
    {
        set_sec_status(CARINFO_VALID);
        return ret;
    }

    ret=combine_data_SS(&session_message_g, output, olen);
    if(ret != INGEEK_OK)
    {
        set_sec_status(CARINFO_VALID);
        return ret;
    }

    memcpy(statem.Keyinfo.SSC,session_message_g.CRnD.bytes,LENGTH_8);
    memcpy(statem.Keyinfo.SSC+LENGTH_8,statem.auth_g.TRnd,LENGTH_8);
    set_sec_status(WRITE_SESSION);

    return INGEEK_OK;
}


int ingeek_command_input_action(unsigned char *input,unsigned int ilen, DK_Cmd_Meg* Can_Packet)
{
    int ret =  0;
    unsigned int oLen = 0;
    ingeek_DK_Cmd cmd_message=ingeek_DK_Cmd_init_zero;

    if(input == NULL || ilen <= 0  || Can_Packet == NULL)
    {
        set_sec_status(CARINFO_VALID);
        return INGEEK_FAILED_PARAM;
    }

    if(ingeek_get_sec_status() != WRITE_SESSION)
    {
        set_sec_status(CARINFO_VALID);
        return INGEEK_FAILED_STATUS;
    }
    memset(uTempBuff,0,MAX_BUFF_LEN);
    ret=ingeek_cipher_aes_cbc(statem.Keyinfo.SK,statem.Keyinfo.SSC,input,ilen,uTempBuff,&oLen, INGEEK_DECRYPT);
    if(ret != INGEEK_OK)
    {
        set_sec_status(CARINFO_VALID);
        return INGEEK_FAILED_DECRYPT;
    }

    ret=PBDecode_client_data(uTempBuff,oLen,&cmd_message,ingeek_DK_Cmd_fields);
    if(ret != INGEEK_OK  || cmd_message.sparam.size > SPARAM_MAX_LEN)
    {
        set_sec_status(CARINFO_VALID);
        return INGEEK_FAILED_PBDECODE;
    }

    Can_Packet->command = cmd_message.command;
    Can_Packet->result = cmd_message.result;
    Can_Packet->index = cmd_message.index;
    Can_Packet->permission = cmd_message.permission;
    Can_Packet->sparam_size=cmd_message.sparam.size;
    memcpy(Can_Packet->sparam,cmd_message.sparam.bytes,cmd_message.sparam.size);

    //DebugPrintf("input: cmd:%d, result:%d, index:%d, permission:%d\n",
    //		Can_Packet->command,Can_Packet->result,Can_Packet->index,Can_Packet->permission);
    //hex_dump("sparam", Can_Packet->sparam, Can_Packet->sparam_size);

    return ret;
}


int ingeek_command_output_action(DK_Cmd_Meg* Can_Packet, unsigned char* output, unsigned int* olen)
{
    int  ret=0;

    ingeek_DK_Cmd cmd_message = ingeek_DK_Cmd_init_zero;
    if(ingeek_get_sec_status() != WRITE_SESSION)
    {
        set_sec_status(CARINFO_VALID);
        memset(statem.Keyinfo.SK, 0x00, LENGTH_16);
        memset(statem.Keyinfo.SSC, 0x00, LENGTH_16);
        return INGEEK_FAILED_STATUS;
    }

    if(Can_Packet->sparam_size > SPARAM_MAX_LEN  || Can_Packet->sparam == NULL)
    {
        set_sec_status(CARINFO_VALID);
        return INGEEK_FAILED_PARAM;
    }

    //DebugPrintf("output:cmd:%d, result:%d, index:%d, permission:%d\n",
    //Can_Packet->command,Can_Packet->result,Can_Packet->index,Can_Packet->permission);
    //hex_dump("sparam", Can_Packet->sparam, Can_Packet->sparam_size);

    cmd_message.command = Can_Packet->command;
    cmd_message.result = Can_Packet->result;
    cmd_message.index = Can_Packet->index;
    cmd_message.permission = Can_Packet->permission;
    cmd_message.sparam.size=Can_Packet->sparam_size;
    memcpy(cmd_message.sparam.bytes, Can_Packet->sparam, Can_Packet->sparam_size);

    unsigned int TempLen = PBEncode_local_data(uTempBuff, MAX_BUFF_LEN, &cmd_message, ingeek_DK_Cmd_fields);
    if(TempLen == INGEEK_FAILED)
    {
        set_sec_status(CARINFO_VALID);
        memset(statem.Keyinfo.SK, 0x00, LENGTH_16);
        memset(statem.Keyinfo.SSC, 0x00, LENGTH_16);
        return INGEEK_FAILED_PBENCODE;
    }

    ret=ingeek_cipher_aes_cbc(statem.Keyinfo.SK,statem.Keyinfo.SSC, uTempBuff, TempLen, output,olen,INGEEK_ENCRYPT);
    if(ret != INGEEK_OK)
    {
        set_sec_status(CARINFO_VALID);
        memset(statem.Keyinfo.SK, 0x00, LENGTH_16);
        memset(statem.Keyinfo.SSC, 0x00, LENGTH_16);
        return INGEEK_FAILED_ENCRYPT;
    }

    return INGEEK_OK;
}


int ingeek_message_output_action(DK_Cmd_Meg* Can_Packet, unsigned char* output, unsigned int* outlen)
{
    unsigned char iv[16]={0};
    ingeek_DK_Msg msg_message=ingeek_DK_Msg_init_zero;

    if(Can_Packet->sparam_size > SPARAM_MAX_LEN  /*|| Can_Packet->sparam == NULL*/)
    {
        //set_sec_status(CARINFO_VALID);
        return INGEEK_FAILED_PARAM;
    }

    msg_message.message = Can_Packet->command;
    msg_message.has_result = true;
    msg_message.result = Can_Packet->result;
    msg_message.has_index = true;
    msg_message.index = Can_Packet->index;
    msg_message.has_permission = true;
    msg_message.permission = Can_Packet->permission;
    msg_message.has_sparam = true;
    msg_message.sparam.size=Can_Packet->sparam_size;
    memcpy(msg_message.sparam.bytes, Can_Packet->sparam, Can_Packet->sparam_size);

    //DebugPrintf("Msg output:cmd:%d, result:%d, index:%d, permission:%d\n",
    //	      Can_Packet->command,Can_Packet->result,Can_Packet->index,Can_Packet->permission);
    //hex_dump("sparam", Can_Packet->sparam, Can_Packet->sparam_size);

    unsigned int TempLen = PBEncode_local_data(uTempBuff,MAX_BUFF_LEN,&msg_message,ingeek_DK_Msg_fields);
    if(TempLen == INGEEK_FAILED)
       return INGEEK_FAILED_PBENCODE;
    
    if(0 != ingeek_cipher_aes_cbc(statem.Keyinfo.SK, iv, uTempBuff, TempLen, output, outlen, INGEEK_ENCRYPT))
       return INGEEK_FAILED_ENCRYPT;
    
    return INGEEK_OK;
}


int ingeek_se_final(void)
{
    if(ingeek_get_sec_status() == CARINFO_INVALID)
    {
        memset(&statem,0x00,sizeof(Ingeek_StateM_t));
        set_sec_status(CARINFO_INVALID);
        return INGEEK_FAILED_STATUS;
    }
    else
    {
        set_sec_status(READ_INFO);//CARINFO_VALID
    }

    return 0;
}

void ingeek_set_callback(read_CallBack rcb,write_CallBack wcb,Rand_CallBack randcb)
{
    g_readcb =  rcb;
    g_writecb = wcb;
    g_randcb =  randcb;
}


static int ingeek_check_info(void)
{
    memset(&statem,0x00,sizeof(Ingeek_StateM_t));
    set_sec_status(CARINFO_INVALID);
    if(g_readcb == NULL)
        return INGEEK_FAILED_NULL;
		
    g_readcb(s_psCalibration,CAL_NUMBER* (BT_MAC_LEN + BT_PSPE_LEN),PSCAL_OFFSET);
    //NRF_LOG_INFO("read pspe ");
   // NRF_LOG_HEXDUMP_INFO((uint8_t *)s_psCalibration, (uint16_t)24);
		
		
    if(g_readcb(statem.vin,VIN_LEN,VIN_OFFSET) != INGEEK_OK || g_readcb(uTempBuff,LENGTH_48,KEY_OFFSET) != INGEEK_OK)
        return INGEEK_FAILED_READCB;
    
     memset(statem.Keyinfo.SSC,0x00,LENGTH_16);
     unsigned int keylen = 0;
     if(ingeek_cipher_aes_cbc(statem.vin,statem.Keyinfo.SSC,uTempBuff,LENGTH_48,statem.Keyinfo.CMPK,&keylen,INGEEK_DECRYPT) != INGEEK_OK \
                    || keylen != LENGTH_32)
          return INGEEK_FAILED_READCB;
     set_sec_status(CARINFO_VALID);
     return INGEEK_OK;
}

int ingeek_se_init(void)
{
 #ifdef IK_DYNA_MALLOC
    if(!uTempBuff)
      uTempBuff = (unsigned char*)malloc( MAX_BUFF_LEN);
 #endif
    set_sec_status(CARINFO_INVALID);
    return ingeek_check_info();	 
}


void ingeek_set_seid(unsigned char*seid)
{
    memcpy(statem.auth_g.SEID,seid,LENGTH_16);
}

unsigned char* ingeek_get_seid(void)
{
    return statem.auth_g.SEID;
}


static void set_sec_status(Sec_status status)
{
    statem.status = status;
}


int ingeek_get_sec_status(void)
{
    return statem.status;
}


unsigned char* ingeek_get_version(void)
{
    return version;
}


unsigned int ingeek_get_permission(void)
{
    return statem.auth_g.KPRE;
}


unsigned int ingeek_get_pair_df(void)
{
    int ret = 0;
    unsigned char hmacout[LENGTH_32]={0};
    unsigned char cmpk[LENGTH_32]={0xe8,0x91,0xc2,0x9e,0x90,0x10,0x84,0x58,0x05,0x58,0x3e,0x23,0x50,0x94,0xca,0x23,
                                   0x68,0x4d,0x33,0x18,0xea,0x96,0xeb,0x8f,0x07,0xfd,0x6e,0xb9,0xbd,0xaf,0xdb,0x27};
    unsigned char   vin1[17]={0x49,0x4e,0x47,0x45,0x45,0x4b,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    const ingeek_md_info_t *md_info=NULL;

    md_info = Get_md_info_from_type(INGEEK_MD_SHA256);
    if( md_info == NULL )
        return INGEEK_FAILED_MDINFO;

    if(ingeek_get_sec_status() == CARINFO_INVALID)
        ret= ingeek_message_digest_hmac(md_info,cmpk,LENGTH_32,vin1,VIN_LEN,hmacout);
    else
        ret= ingeek_message_digest_hmac(md_info,statem.Keyinfo.CMPK,LENGTH_32,statem.vin,VIN_LEN,hmacout);

    if(ret != 0)
        return INGEEK_FAILED_HMAC;
    return ((hmacout[0]<<24)+ (hmacout[1]<<16)+ (hmacout[2]<<8)+ hmacout[3]);
}


unsigned int ingeek_calculate_vinhash(unsigned char* output, unsigned int* outlen)
{
    unsigned char mdout[LENGTH_32]={0};
    const ingeek_md_info_t *md_info=NULL;

    if( (md_info = Get_md_info_from_type(INGEEK_MD_SHA256)) == NULL )
        return INGEEK_FAILED_MDINFO;

    *outlen = 16;
    if(ingeek_message_digest(md_info,statem.vin,sizeof(unsigned int),mdout) != 0)
        return INGEEK_FAILED_MD;

    memcpy(output,mdout,16);
    return INGEEK_OK;
}

int ingeek_SaveCalibration()
{
	  if(g_writecb)
		  return g_writecb(s_psCalibration,CAL_NUMBER* (BT_MAC_LEN + BT_PSPE_LEN) , PSCAL_OFFSET);
		
	  return true;
}

int ingeek_SetPSCalibration(const unsigned char * bleMac,unsigned char psValue,unsigned char peValue)
{
	 int offset = 0;
	 if((s_psCalibration[0] == 0xff && s_psCalibration[1] == 0xff)\
		 ||(s_psCalibration[0] == 0x0 && s_psCalibration[1] == 0x0)) 
		  offset = 0;
	 else if((s_psCalibration[8] == 0xff && s_psCalibration[9] == 0xff)\
		 ||(s_psCalibration[8] == 0x0 && s_psCalibration[9] == 0x0))
         offset = 8;
	  if((s_psCalibration[16] == 0xff && s_psCalibration[17] == 0xff)\
		 ||(s_psCalibration[16] == 0x0 && s_psCalibration[17] == 0x0)) 
		 offset = 16;
		
	  if(bleMac)
       memcpy(s_psCalibration+offset,bleMac,BT_MAC_LEN);
    
	 s_psCalibration[offset+BT_MAC_LEN] = psValue;
	 s_psCalibration[offset+BT_MAC_LEN+1] = peValue;

	 //NRF_LOG_INFO("save cal %d =%d %d ", offset,s_psCalibration[offset+BT_MAC_LEN], s_psCalibration[1+offset+BT_MAC_LEN]);

		return true;
}

int ingeek_GetPSCalibration(const unsigned char * bleMac,unsigned char * psValue,unsigned char * peValue)
{
	 int offset = 0;
	 if(bleMac && memcmp(s_psCalibration,bleMac,BT_MAC_LEN) == 0)
		  offset = 0;
	 else if(bleMac && memcmp(s_psCalibration+8,bleMac,BT_MAC_LEN) == 0)
		  offset = 8;
	 else if(bleMac &&memcmp(s_psCalibration+16,bleMac,BT_MAC_LEN) == 0)
		  offset = 16;
	 
	 if(psValue)
		  *psValue = s_psCalibration[offset+BT_MAC_LEN] ;
	 
	 if(peValue)
		  *peValue = s_psCalibration[offset+BT_MAC_LEN+1] ;

	 return 1;
}


#if 0

static unsigned char UTC_monthLength(unsigned char lpyr, unsigned char mon)
{
    unsigned char days = 31;

    if (mon == 1){// feb
        days = (28 + lpyr);
    }
    else{
        if (mon > 6){// aug-dec
            mon--;
        }

        if (mon & 1){
            days = 30;
        }
    }
    return (days);
}


static void UTC_convertUTCTime(UTCTimeStruct *tm, UTCTime secTime)
{
    // Calculate the time less than a day - hours, minutes, seconds.
    {
        // The number of seconds that have occured so far stoday.
        unsigned int day = secTime % DAY;

        // Seconds that have passed in the current minute.
        tm->seconds = day % 60UL;
        // Minutes that have passed in the current hour.
        // (seconds per day) / (seconds per minute) = (minutes on an hour boundary)
        tm->minutes = (day % 3600UL) / 60UL;
        // Hours that have passed in the current day.
        tm->hour = day / 3600UL;
    }

    // Fill in the calendar - day, month, year
    {
        unsigned short numDays = secTime / DAY;
        unsigned char monthLen;
        tm->year = BEGYEAR;

        while (numDays >= YearLength(tm->year))
        {
            numDays -= YearLength(tm->year);
            tm->year++;
        }

        // January.
        tm->month = 0;

        monthLen = UTC_monthLength(IsLeapYear(tm->year), tm->month);

        // Determine the number of months which have passed from remaining days.
        while (numDays >= monthLen)
        {
            // Subtract number of days in month from remaining count of days.
            numDays -= monthLen;
            tm->month++;

            // Recalculate month length.
            monthLen = UTC_monthLength(IsLeapYear(tm->year), tm->month);
        }

        // Store the remaining days.
        tm->day = numDays;
    }
}


int ingeek_set_RTC(unsigned char *input, unsigned int ilen, UTCTimeStruct* utc_time)
{
    int ret = 0;
    ret=Handle_RTC_info(input,ilen,utc_time);
    return ret;
}


static int Handle_RTC_info(unsigned char *input,unsigned int ilen,UTCTimeStruct* utc_time)
{
    int ret=0;
    ingeek_DK_RTC message = ingeek_DK_RTC_init_zero;

    ret=PBDecode_client_data(input,ilen,&message,ingeek_DK_RTC_fields);
    if(ret != INGEEK_OK)
    {
        return INGEEK_FAILED_PBDECODE;
    }

    if(message.rtc_time.seconds <= 0)
    {
        return INGEEK_FAILED_RTC;
    }
    else
    {
        UTCTime u_time = message.rtc_time.seconds;
        UTC_convertUTCTime(&utc_time, u_time);
    }

    return INGEEK_OK;
}


static int Handle_whitelist(ingeek_DK_InfoW* info_message)
{
    int ret = 0;
    int index=info_message->index;

    if(index<=-16 || index>=16)
        return INGEEK_ERR_INDEX_INVALID;

    if(whitelist[index]!=NULL) //ÒÑ±»Ìî³ä¹ý
    {
        return INGEEK_ERR_INDEX_EXIST;
    }

    if(index>=0 && index<16) //add record
    {
        whitelist[index] = (unsigned char*)malloc(info_message->KEY.size);
        if(whitelist[index] == NULL)
        {

            return INGEEK_ERR_MALLOC;
        }
        memcpy(whitelist[index], info_message->KEY.bytes, info_message->KEY.size);
    }
    else if(index>-16 && index<0)
    {
        if(whitelist[info_message->index])
        {
            free(whitelist[info_message->index]);
            whitelist[info_message->index] = NULL;
        }
    }
    return ret;
}


int ingeek_push_whitelist(unsigned char *input, int ilen)
{
    int ret=Handle_write_data(g_writecb,input,ilen);
    if(ret == INGEEK_OK)
    {
        set_sec_status(CARINFO_VALID);
    }
    else
    {
        memset(statem.vin,0x00,sizeof(statem.vin));//clear vin
        set_sec_status(READ_INFO);
    }
    return ret;
}
#endif

/*********************************************************************
*********************************************************************/

