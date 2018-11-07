/**
* @file         security_digital_key.h
* @brief        ���ļ����ڷ�װ����Ӧ�ó����м��ܺ�����ҵ��
* @author       ingeek telematics securtiy team
* @date         2018-4-17
* @version      dk-v1.0
* @copyright    Ingeek Information Security Consulting Associates Co.
*/

#ifndef DIGITAL_KEY_API_H
#define DIGITAL_KEY_API_H

//#include "DigitalKey.pb.h"


#ifdef __cplusplus
extern "C"
{
#endif

typedef unsigned int UTCTime;

/*
typedef   signed char   int8_t;
typedef  int8_t   int_least8_t;
typedef uint8_t  uint_least8_t;


#if defined(PB_FIELD_32BIT)
    typedef uint32_t pb_size_t;
    typedef int32_t pb_ssize_t;
#elif defined(PB_FIELD_16BIT)
    typedef uint_least16_t pb_size_t;
    typedef int_least16_t pb_ssize_t;
#else
    typedef uint_least8_t pb_size_t;
    typedef int_least8_t pb_ssize_t;
#endif

typedef uint_least8_t pb_byte_t;

#define	bool	_Bool
#if __TI_STRICT_ANSI_MODE__ && 199901L > __STDC_VERSION__
typedef unsigned char _Bool;
#endif



#define PB_BYTES_ARRAY_T(n) struct{ pb_size_t size; pb_byte_t bytes[n]; }
typedef PB_BYTES_ARRAY_T(20) ingeek_DK_Cmd_sparam_t;
typedef struct _ingeek_DK_Cmd {
    int32_t command;
    int32_t result;
    bool has_wparam;
    int32_t wparam;
    bool has_sparam;
    ingeek_DK_Cmd_sparam_t sparam;

} ingeek_DK_Cmd;*/

typedef struct DK_Cmd {
    int32_t command;
    int32_t result;
    int32_t index;
    int32_t permission;
    uint8_t sparam[20];
} DK_Cmd;

typedef struct
{
  uint8_t seconds;  // 0-59
  uint8_t minutes;  // 0-59
  uint8_t hour;     // 0-23
  uint8_t day;      // 0-30
  uint8_t month;    // 0-11
  uint16_t year;    // 2000+
} UTCTimeStruct;

#define BEGYEAR  1970     //  UTC started at 00:00:00 January 1, 2000
#define DAY      86400UL  // 24 hours * 60 minutes * 60 seconds
#define YearLength(yr)  (IsLeapYear(yr) ? 366 : 365)
#define IsLeapYear(yr)  (!((yr) % 400) || (((yr) % 100) && !((yr) % 4)))


typedef int (*read_CallBack)(unsigned char *out, unsigned int rlen, unsigned int offset);
typedef int (*write_CallBack)(unsigned char *in, unsigned int wlen, unsigned int offset);
typedef int (*Rand_CallBack)(void *p_rng, unsigned char *rand, unsigned int randlen);

extern void ingeek_set_callback(read_CallBack,write_CallBack,Rand_CallBack randcb);

extern int ingeek_se_final(void);
 
extern int ingeek_se_init(void);

extern int ingeek_pull_info(unsigned char *output,unsigned int* olen);

extern int ingeek_push_info(unsigned char *input, int ilen);
extern int ingeek_push_auth(unsigned char *input, unsigned int ilen, unsigned char* output, unsigned int* olen);

extern int ingeek_push_session(unsigned char *input,unsigned int ilen,unsigned char* output, unsigned int* olen);

extern int ingeek_get_sec_status(void);

extern unsigned char* ingeek_get_version(void);

extern void ingeek_set_seid(unsigned char *seid);

extern unsigned int ingeek_get_permission(void);

//extern int ingeek_set_RTC(unsigned char *input,unsigned int size, UTCTimeStruct* utc_time);

//extern int ingeek_push_whitelist(unsigned char *input, int ilen);

extern int ingeek_get_pair_df(void);

extern int ingeek_command_output_action(DK_Cmd* struct_cmd, unsigned char* output, unsigned int* olen);

extern int ingeek_command_input_action(unsigned char *input,unsigned int ilen, DK_Cmd* struct_cmd);

//extern int ingeek_message_output_action(ingeek_DK_Msg* struct_msg, uint8_t* output, uint32_t* outlen);

extern int ingeek_calculate_vinhash(uint8_t* output, uint32_t* outlen);

/**@name    ��������
* @{
*/
#define SUCCESS 0
#define FAILURE -1
#define INGEEK_ERR_PBDECODE        -0x0005   ///<Protobuf decode��������
#define INGEEK_ERR_WRITE_INFO      -0x0006   ///<SNVд��������
#define INGEEK_ERR_AES_DECRYPT     -0x0007   ///<AES���ܳ���
#define INGEEK_ERR_MALLOC          -0x0008   ///<�ڴ�������
#define INGEEK_ERR_VERIFY_SESSION  -0x0009   ///<��֤session����ֵ����
#define INGEEK_ERR_ENCODE_CMD      -0x0010   ///<Protobuf encode command����ֵ����
#define INGEEK_ERR_CNAME           -0x0011   ///<auth��session����ֵ�ṹ����name��ȶԳ���
#define INGEEK_ERR_CREAM           -0x0012   ///<auth��session����ֵ�ṹ����realm��ȶԳ���
#define INGEEK_ERR_AUTH            -0x0013   ///<auth��session����ֵ�ṹ����Ȩ��ֵ�ȶԳ���
#define INGEEK_ERR_CTIME           -0x0014   ///<auth��session����ֵ�ṹ����ctime�ȶԳ���
#define INGEEK_ERR_PBENCODE        -0x0015   ///<protobuf encode��������
#define INGEEK_ERR_AES_ENCRYPT     -0x0016   ///<AES���ܳ���
#define INGEEK_ERR_MD_INFO_TYPE    -0x0017   ///<MD��֧�ֵ�ʹ������
#define INGEEK_ERR_MD_HMAC         -0x0018   ///<����HMACʱ�����
#define INGEEK_ERR_SHA256          -0x0019   ///<ʹ��HASH����
#define INGEEK_ERR_SEID            -0x0020   ///<SEID��ȶԳ���
#define INGEEK_ERR_TRND            -0x0021   ///<TRND��ȶԳ���
#define INGEEK_ERR_CALCULATE_VCK   -0x0022   ///<����VCK����
#define INGEEK_ERR_AESCMAC_VERIFY  -0x0023   ///<AES-CMACУ�����
#define INGEEK_ERR_CIPHER_SET_UP   -0x0025
#define INGEEK_ERR_CIPHER_SET_KEY  -0x0026
#define INGEEK_ERR_CIPHER_CRYPT    -0x0027
#define INGEEK_ERR_INDEX_INVALID   -0x0028
#define INGEEK_ERR_INDEX_EXIST     -0x0029
#define INGEEK_ERR_RTC_INVALID     -0x002A
#define INGEEK_ERR_WRITE_WHITELIST -0x002B
#define INGEEK_ERR_WRITE_VIN       -0x002C   ///<SNVд��������
#define INGEEK_ERR_WRITE_KEY       -0x002D   ///<SNVд��������
#define INGEEK_ERR_WRITE_CALC_SK   -0x002E   
#define INGEEK_ERR_WRITE_SEND_SS   -0x002F   



/** @}*/

#ifdef __cplusplus
}
#endif

#endif
