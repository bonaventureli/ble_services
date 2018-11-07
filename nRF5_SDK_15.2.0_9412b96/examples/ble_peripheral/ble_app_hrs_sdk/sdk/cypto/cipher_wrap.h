
#ifndef INGEEK_CIPHER_WRAP_H
#define INGEEK_CIPHER_WRAP_H

#if !defined(INGEEK_CONFIG_FILE)
#include "config.h"
#else
#include INGEEK_CONFIG_FILE
#endif

#if !defined(INGEEK_SMALL_CIPHER_C)

#include "cipher_export.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * Base cipher information. The non-mode specific functions and values.
 */
struct ingeek_cipher_base_t
{
    /** Base Cipher type (e.g. INGEEK_CIPHER_ID_AES) */
    ingeek_cipher_id_t cipher;

    /** Encrypt using ECB */
    int (*ecb_func)( void *ctx, ingeek_operation_t mode,
                     const unsigned char *input, unsigned char *output );

#if defined(INGEEK_CIPHER_MODE_CBC)
    /** Encrypt using CBC */
    int (*cbc_func)( void *ctx, ingeek_operation_t mode, size_t length,
                     unsigned char *iv, const unsigned char *input,
                     unsigned char *output );
#endif

    /** Set key for encryption purposes */
    int (*setkey_enc_func)( void *ctx, const unsigned char *key,
                            unsigned int key_bitlen );

    /** Set key for decryption purposes */
    int (*setkey_dec_func)( void *ctx, const unsigned char *key,
                            unsigned int key_bitlen);

    /** Allocate a new context */
    void * (*ctx_alloc_func)( void );

    /** Free the given context */
    void (*ctx_free_func)( void *ctx );

};


typedef struct
{
    ingeek_cipher_type_t type;
    const ingeek_cipher_info_t *info;
} ingeek_cipher_definition_t;

extern const ingeek_cipher_definition_t ingeek_cipher_definitions[];

extern int ingeek_cipher_supported[];

#endif /*INGEEK_SMALL_CIPHER_C*/

#ifdef __cplusplus
}
#endif

#endif /* INGEEK_CIPHER_WRAP_H */

