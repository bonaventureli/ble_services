#ifndef _INGEEK_CMAC_H
#define _INGEEK_CMAC_H

#ifdef __cplusplus
    extern "C" {
#endif


#define AES_BLOCK_SIZE 16

#define min(a,b) (((a)<=(b))?(a):(b))

/**
 * \brief          Output = message_MAC
 *
 * \param out  	  message MAC
 * \param outSz   The MAC value size
 * \param in      The input data
 * \param inSz    Length of the input data
 * \param key     The key to use.
 * \param keySz   key length to use,
 *
 * \returns        0 on success
 */
int ingeek_AesCmacGenerate(unsigned char* out, int* outSz,
                       const unsigned char* in, int inSz,
                       const unsigned char* key, int keySz);

/**
 * \brief          Output = checksum result
 *
 * \param check    The MAC to use.
 * \param checkSz  The MAC value size
 * \param in       The input data
 * \param inSz     Length of the input data
 * \param key      The key to use.
 * \param keySz    key length to use,
 
 * \returns        0 on success
 */
int ingeek_AesCmacVerify(const unsigned char* check, int checkSz,
                     const unsigned char* in, int inSz,
                     const unsigned char* key, int keySz);

#ifdef __cplusplus
    }
#endif

#endif 

