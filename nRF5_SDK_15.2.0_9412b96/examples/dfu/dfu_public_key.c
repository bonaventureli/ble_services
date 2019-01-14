
/* This file was automatically generated by nrfutil on 2018-12-26 (YY-MM-DD) at 17:03:26 */

#include "sdk_config.h"
#include "stdint.h"
#include "compiler_abstraction.h"

#if NRF_CRYPTO_BACKEND_OBERON_ENABLED
/* Oberon backend is changing endianness thus public key must be kept in RAM. */
#define _PK_CONST
#else
#define _PK_CONST const
#endif


/** @brief Public key used to verify DFU images */
__ALIGN(4) _PK_CONST uint8_t pk[64] =
{
    0xeb, 0xf3, 0x7b, 0xc8, 0xe2, 0x74, 0xd4, 0xbf, 0xcd, 0x14, 0x99, 0x7a, 0x3a, 0x79, 0x5a, 0x49, 0xe6, 0x4d, 0x6b, 0x80, 0x4f, 0xb2, 0x6e, 0x5b, 0xca, 0xe0, 0xff, 0x34, 0x0e, 0xc4, 0xb3, 0xe6, 
    0xfe, 0x5e, 0x8b, 0xa3, 0xba, 0xe0, 0xe2, 0x15, 0x4e, 0xef, 0x8b, 0x3f, 0x6b, 0xb1, 0x63, 0x21, 0x89, 0x19, 0x84, 0xec, 0x6b, 0x95, 0xd8, 0xd9, 0x01, 0x56, 0xd6, 0x72, 0xdf, 0xbb, 0xcd, 0x34
};
