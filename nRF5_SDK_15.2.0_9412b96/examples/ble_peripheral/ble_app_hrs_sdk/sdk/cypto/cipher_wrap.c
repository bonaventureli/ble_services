
#if !defined(INGEEK_CONFIG_FILE)
#include "config.h"
#else
#include INGEEK_CONFIG_FILE
#endif

#if !defined(INGEEK_SMALL_CIPHER_C)

#include "cipher_wrap.h"

#if defined(INGEEK_AES_C)
#include "aes.h"
#endif

#if defined(INGEEK_PLATFORM_C)
#include "ingeek/platform.h"
#else
#include <stdlib.h>
#define ingeek_calloc    calloc
#define ingeek_free       free
#endif



#if defined(INGEEK_AES_C)

static int aes_crypt_ecb_wrap( void *ctx, ingeek_operation_t operation,
        const unsigned char *input, unsigned char *output )
{
    return ingeek_aes_crypt_ecb( (ingeek_aes_context *) ctx, operation, input, output );
}

#if defined(INGEEK_CIPHER_MODE_CBC)
static int aes_crypt_cbc_wrap( void *ctx, ingeek_operation_t operation, size_t length,
        unsigned char *iv, const unsigned char *input, unsigned char *output )
{
    return ingeek_aes_crypt_cbc( (ingeek_aes_context *) ctx, operation, length, iv, input,
                          output );
}
#endif /* INGEEK_CIPHER_MODE_CBC */

static int aes_setkey_dec_wrap( void *ctx, const unsigned char *key,
                                unsigned int key_bitlen )
{
    return ingeek_aes_setkey_dec( (ingeek_aes_context *) ctx, key, key_bitlen );
}

static int aes_setkey_enc_wrap( void *ctx, const unsigned char *key,
                                unsigned int key_bitlen )
{
    return ingeek_aes_setkey_enc( (ingeek_aes_context *) ctx, key, key_bitlen );
}

static void * aes_ctx_alloc( void )
{
    ingeek_aes_context *aes = ingeek_calloc( 1, sizeof( ingeek_aes_context ) );

    if( aes == NULL )
        return( NULL );

    ingeek_aes_init( aes );

    return( aes );
}

static void aes_ctx_free( void *ctx )
{
    ingeek_aes_free( (ingeek_aes_context *) ctx );
    ingeek_free( ctx );
}

static const ingeek_cipher_base_t aes_info = {
    INGEEK_CIPHER_ID_AES,
    aes_crypt_ecb_wrap,
#if defined(INGEEK_CIPHER_MODE_CBC)
    aes_crypt_cbc_wrap,
#endif
    aes_setkey_enc_wrap,
    aes_setkey_dec_wrap,
    aes_ctx_alloc,
    aes_ctx_free
};

static const ingeek_cipher_info_t aes_128_ecb_info = {
    INGEEK_CIPHER_AES_128_ECB,
    INGEEK_MODE_ECB,
    128,
    "AES-128-ECB",
    16,
    0,
    16,
    &aes_info
};

static const ingeek_cipher_info_t aes_192_ecb_info = {
    INGEEK_CIPHER_AES_192_ECB,
    INGEEK_MODE_ECB,
    192,
    "AES-192-ECB",
    16,
    0,
    16,
    &aes_info
};

static const ingeek_cipher_info_t aes_256_ecb_info = {
    INGEEK_CIPHER_AES_256_ECB,
    INGEEK_MODE_ECB,
    256,
    "AES-256-ECB",
    16,
    0,
    16,
    &aes_info
};

#if defined(INGEEK_CIPHER_MODE_CBC)
static const ingeek_cipher_info_t aes_128_cbc_info = {
    INGEEK_CIPHER_AES_128_CBC,
    INGEEK_MODE_CBC,
    128,
    "AES-128-CBC",
    16,
    0,
    16,
    &aes_info
};

static const ingeek_cipher_info_t aes_192_cbc_info = {
    INGEEK_CIPHER_AES_192_CBC,
    INGEEK_MODE_CBC,
    192,
    "AES-192-CBC",
    16,
    0,
    16,
    &aes_info
};

static const ingeek_cipher_info_t aes_256_cbc_info = {
    INGEEK_CIPHER_AES_256_CBC,
    INGEEK_MODE_CBC,
    256,
    "AES-256-CBC",
    16,
    0,
    16,
    &aes_info
};
#endif /* INGEEK_CIPHER_MODE_CBC */
#endif /* INGEEK_AES_C */


const ingeek_cipher_definition_t ingeek_cipher_definitions[] =
{
#if defined(INGEEK_AES_C)
    { INGEEK_CIPHER_AES_128_ECB,          &aes_128_ecb_info },
    { INGEEK_CIPHER_AES_192_ECB,          &aes_192_ecb_info },
    { INGEEK_CIPHER_AES_256_ECB,          &aes_256_ecb_info },
#if defined(INGEEK_CIPHER_MODE_CBC)
    { INGEEK_CIPHER_AES_128_CBC,          &aes_128_cbc_info },
    { INGEEK_CIPHER_AES_192_CBC,          &aes_192_cbc_info },
    { INGEEK_CIPHER_AES_256_CBC,          &aes_256_cbc_info },
#endif
#endif /* INGEEK_AES_C */
    { INGEEK_CIPHER_NONE, NULL }
};

#define NUM_CIPHERS sizeof ingeek_cipher_definitions / sizeof ingeek_cipher_definitions[0]
int ingeek_cipher_supported[NUM_CIPHERS];

#endif /* INGEEK_SMALL_CIPHER_C */
