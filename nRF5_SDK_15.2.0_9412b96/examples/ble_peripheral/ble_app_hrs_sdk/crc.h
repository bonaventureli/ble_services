/******************************************************************************

******************************************************************************/
#ifndef __CRC16_H__
#define __CRC16_H__


#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#define CRC_CAL_TABLE_MODE 0


#if CRC_CAL_TABLE_MODE

uint16_t crc16_cal_ccitt_table(uint8_t* dataIn, uint32_t length);

uint16_t crc16_cal_ibm_table(uint8_t* dataIn, uint32_t length);


#else

uint16_t crc16_cal_ccitt(uint8_t *puchMsg, uint32_t usDataLen);

uint16_t crc16_cal_ibm(uint8_t *puchMsg, uint32_t usDataLen);


uint16_t lcrc16_compute(uint8_t const * p_data, uint32_t size, uint16_t const * p_crc);
bool lcrc_verify_success(uint16_t crc, uint16_t len_words, uint8_t const * const p_data);
#endif

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */


#endif /* __CRC16_H__ */
