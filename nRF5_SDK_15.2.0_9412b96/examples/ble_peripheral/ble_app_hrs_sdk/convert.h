#ifndef __CONVERT_H__
#define __CONVERT_H__


#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */

#include <stdint.h>
#include <stdio.h>
#include <string.h>


uint8_t Escape_frame_data(uint8_t* input, uint32_t ilen, uint8_t* output, uint32_t* olen);
uint8_t Reduction_frame_data(uint8_t* input, uint32_t ilen, uint8_t* output, uint32_t* olen);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */


#endif /* __CONVERT_H__ */
