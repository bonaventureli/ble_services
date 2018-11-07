#ifndef INGEEK_CIPHER_H
#define INGEEK_CIPHER_H


#if !defined(INGEEK_CONFIG_FILE)
#include "config.h"
#else
#include INGEEK_CONFIG_FILE
#endif


typedef enum {
    INGEEK_OPERATION_NONE = -1,
    INGEEK_DECRYPT = 0,
    INGEEK_ENCRYPT,
} ingeek_operation_t;


#if !defined(INGEEK_SMALL_CIPHER_C)

#include <stddef.h>

#define INGEEK_CIPHER_VARIABLE_IV_LEN     				 0x01     /**< Cipher accepts IVs of variable length */
#define INGEEK_CIPHER_VARIABLE_KEY_LEN    				 0x02     /**< Cipher accepts keys of variable length */
#define INGEEK_ERR_CIPHER_FEATURE_UNAVAILABLE            -0x6080  /**< The selected feature is not available. */
#define INGEEK_ERR_CIPHER_BAD_INPUT_DATA                 -0x6100  /**< Bad input parameters to function. */
#define INGEEK_ERR_CIPHER_ALLOC_FAILED                   -0x6180  /**< Failed to allocate memory. */
#define INGEEK_ERR_CIPHER_INVALID_PADDING                -0x6200  /**< Input data contains invalid padding and is rejected. */
#define INGEEK_ERR_CIPHER_FULL_BLOCK_EXPECTED            -0x6280  /**< Decryption of block requires a full block. */
#define INGEEK_ERR_CIPHER_AUTH_FAILED                    -0x6300  /**< Authentication failed (for AEAD modes). */
#define INGEEK_ERR_CIPHER_INVALID_CONTEXT                -0x6380  /**< The context is invalid, eg because it was free()ed. */

#ifdef __cplusplus
extern "C" {
#endif


typedef enum {
    INGEEK_CIPHER_ID_NONE = 0,
    INGEEK_CIPHER_ID_NULL,
    INGEEK_CIPHER_ID_AES,
} ingeek_cipher_id_t;


typedef enum {
    INGEEK_CIPHER_NONE = 0,
    INGEEK_CIPHER_NULL,
    INGEEK_CIPHER_AES_128_ECB,
    INGEEK_CIPHER_AES_192_ECB,
    INGEEK_CIPHER_AES_256_ECB,
    INGEEK_CIPHER_AES_128_CBC,
    INGEEK_CIPHER_AES_192_CBC,
    INGEEK_CIPHER_AES_256_CBC
} ingeek_cipher_type_t;


typedef enum {
    INGEEK_MODE_NONE = 0,
    INGEEK_MODE_ECB,
    INGEEK_MODE_CBC
} ingeek_cipher_mode_t;


typedef enum {
    INGEEK_PADDING_PKCS7 = 0,     /**< PKCS7 padding (default)        */
    INGEEK_PADDING_ONE_AND_ZEROS, /**< ISO/IEC 7816-4 padding         */
    INGEEK_PADDING_ZEROS_AND_LEN, /**< ANSI X.923 padding             */
    INGEEK_PADDING_ZEROS,         /**< zero padding (not reversible!) */
    INGEEK_PADDING_NONE,          /**< never pad (full blocks only)   */
} ingeek_cipher_padding_t;


#if defined(INGEEK_CIPHER_MODE_CBC)
#define INGEEK_CIPHER_MODE_WITH_PADDING
#endif
/** Maximum length of any IV, in bytes */
#define INGEEK_MAX_IV_LENGTH      16
/** Maximum block size of any cipher, in bytes */
#define INGEEK_MAX_BLOCK_LENGTH   16

/**
 * Base cipher information (opaque struct).
 */
typedef struct ingeek_cipher_base_t ingeek_cipher_base_t;

/**
 * Cipher information. Allows cipher functions to be called in a generic way.
 */
typedef struct {
    ingeek_cipher_type_t type;
    ingeek_cipher_mode_t mode;
    unsigned int key_bitlen;
    const char * name;
    unsigned int iv_size;
    int flags;
    unsigned int block_size;
    const ingeek_cipher_base_t *base;
} ingeek_cipher_info_t;

/**
 * Generic cipher context.
 */
typedef struct {
    const ingeek_cipher_info_t *cipher_info;
    int key_bitlen;
    ingeek_operation_t operation;
#if defined(INGEEK_CIPHER_MODE_WITH_PADDING)
    void (*add_padding)( unsigned char *output, size_t olen, size_t data_len );
    int (*get_padding)( unsigned char *input, size_t ilen, size_t *data_len );
#endif
    unsigned char unprocessed_data[INGEEK_MAX_BLOCK_LENGTH];
    size_t unprocessed_len;
    unsigned char iv[INGEEK_MAX_IV_LENGTH];
    size_t iv_size;
    void *cipher_ctx;
} ingeek_cipher_context_t;


/**
 * \brief               Returns the cipher information structure associated
 *                      with the given cipher type.
 *
 * \param cipher_type   Type of the cipher to search for.
 *
 * \return              the cipher information structure associated with the
 *                      given cipher_type, or NULL if not found.
 */
const ingeek_cipher_info_t *Get_cipher_info_from_type( const ingeek_cipher_type_t cipher_type );


/**
 * \brief               Initialize a cipher_context (as NONE)
 */
void ingeek_cipher_init( ingeek_cipher_context_t *ctx );

/**
 * \brief               Free and clear the cipher-specific context of ctx.
 *                      Freeing ctx itself remains the responsibility of the
 *                      caller.
 */
void ingeek_cipher_free( ingeek_cipher_context_t *ctx );

/**
 * \brief               Initialises and fills the cipher context structure with
 *                      the appropriate values.
 *
 * \note                Currently also clears structure. In future versions you
 *                      will be required to call ingeek_cipher_init() on the structure
 *                      first.
 *
 * \param ctx           context to initialise. May not be NULL.
 * \param cipher_info   cipher to use.
 *
 * \return              0 on success,
 *                      INGEEK_ERR_CIPHER_BAD_INPUT_DATA on parameter failure,
 *                      INGEEK_ERR_CIPHER_ALLOC_FAILED if allocation of the
 *                      cipher-specific context failed.
 */
int ingeek_cipher_setup( ingeek_cipher_context_t *ctx, const ingeek_cipher_info_t *cipher_info );

/**
 * \brief               Set the key to use with the given context.
 *
 * \param ctx           generic cipher context. May not be NULL. Must have been
 *                      initialised using cipher_context_from_type or
 *                      cipher_context_from_string.
 * \param key           The key to use.
 * \param key_bitlen    key length to use, in bits.
 * \param operation     Operation that the key will be used for, either
 *                      INGEEK_ENCRYPT or INGEEK_DECRYPT.
 *
 * \returns             0 on success, INGEEK_ERR_CIPHER_BAD_INPUT_DATA if
 *                      parameter verification fails or a cipher specific
 *                      error code.
 */
int ingeek_cipher_setkey( ingeek_cipher_context_t *ctx, const unsigned char *key,
                   int key_bitlen, const ingeek_operation_t operation );

/**
 * \brief               Set padding mode, for cipher modes that use padding.
 *                      (Default: PKCS7 padding.)
 *
 * \param ctx           generic cipher context
 * \param mode          padding mode
 *
 * \returns             0 on success, INGEEK_ERR_CIPHER_FEATURE_UNAVAILABLE
 *                      if selected padding mode is not supported, or
 *                      INGEEK_ERR_CIPHER_BAD_INPUT_DATA if the cipher mode
 *                      does not support padding.
 */
#if defined(INGEEK_CIPHER_MODE_WITH_PADDING)
int ingeek_cipher_set_padding_mode( ingeek_cipher_context_t *ctx, ingeek_cipher_padding_t mode );
#endif /* INGEEK_CIPHER_MODE_WITH_PADDING */

/**
 * \brief               Set the initialization vector (IV) or nonce
 *
 * \param ctx           generic cipher context
 * \param iv            IV to use (or NONCE_COUNTER for CTR-mode ciphers)
 * \param iv_len        IV length for ciphers with variable-size IV;
 *                      discarded by ciphers with fixed-size IV.
 *
 * \returns             0 on success, or INGEEK_ERR_CIPHER_BAD_INPUT_DATA
 *
 * \note                Some ciphers don't use IVs nor NONCE. For these
 *                      ciphers, this function has no effect.
 */
int ingeek_cipher_set_iv( ingeek_cipher_context_t *ctx,
                   const unsigned char *iv, size_t iv_len );


/**
 * \brief               Generic cipher update function. Encrypts/decrypts
 *                      using the given cipher context. Writes as many block
 *                      size'd blocks of data as possible to output. Any data
 *                      that cannot be written immediately will either be added
 *                      to the next block, or flushed when cipher_final is
 *                      called.
 *                      Exception: for INGEEK_MODE_ECB, expects single block
 *                                 in size (e.g. 16 bytes for AES)
 *
 * \param ctx           generic cipher context
 * \param input         buffer holding the input data
 * \param ilen          length of the input data
 * \param output        buffer for the output data. Should be able to hold at
 *                      least ilen + block_size. Cannot be the same buffer as
 *                      input!
 * \param olen          length of the output data, will be filled with the
 *                      actual number of bytes written.
 *
 * \returns             0 on success, INGEEK_ERR_CIPHER_BAD_INPUT_DATA if
 *                      parameter verification fails,
 *                      INGEEK_ERR_CIPHER_FEATURE_UNAVAILABLE on an
 *                      unsupported mode for a cipher or a cipher specific
 *                      error code.
 *
 * \note                If the underlying cipher is GCM, all calls to this
 *                      function, except the last one before ingeek_cipher_finish(),
 *                      must have ilen a multiple of the block size.
 */
int ingeek_cipher_update( ingeek_cipher_context_t *ctx, const unsigned char *input,
                   size_t ilen, unsigned char *output, size_t *olen );

/**
 * \brief               Generic cipher finalisation function. If data still
 *                      needs to be flushed from an incomplete block, data
 *                      contained within it will be padded with the size of
 *                      the last block, and written to the output buffer.
 *
 * \param ctx           Generic cipher context
 * \param output        buffer to write data to. Needs block_size available.
 * \param olen          length of the data written to the output buffer.
 *
 * \returns             0 on success, INGEEK_ERR_CIPHER_BAD_INPUT_DATA if
 *                      parameter verification fails,
 *                      INGEEK_ERR_CIPHER_FULL_BLOCK_EXPECTED if decryption
 *                      expected a full block but was not provided one,
 *                      INGEEK_ERR_CIPHER_INVALID_PADDING on invalid padding
 *                      while decrypting or a cipher specific error code.
 */
int ingeek_cipher_finish( ingeek_cipher_context_t *ctx,
                   unsigned char *output, size_t *olen );


/**
 * \brief               Generic all-in-one encryption/decryption
 *                      (for all ciphers except AEAD constructs).
 *
 * \param ctx           generic cipher context
 * \param iv            IV to use (or NONCE_COUNTER for CTR-mode ciphers)
 * \param iv_len        IV length for ciphers with variable-size IV;
 *                      discarded by ciphers with fixed-size IV.
 * \param input         buffer holding the input data
 * \param ilen          length of the input data
 * \param output        buffer for the output data. Should be able to hold at
 *                      least ilen + block_size. Cannot be the same buffer as
 *                      input!
 * \param olen          length of the output data, will be filled with the
 *                      actual number of bytes written.
 *
 * \note                Some ciphers don't use IVs nor NONCE. For these
 *                      ciphers, use iv = NULL and iv_len = 0.
 *
 * \returns             0 on success, or
 *                      INGEEK_ERR_CIPHER_BAD_INPUT_DATA, or
 *                      INGEEK_ERR_CIPHER_FULL_BLOCK_EXPECTED if decryption
 *                      expected a full block but was not provided one, or
 *                      INGEEK_ERR_CIPHER_INVALID_PADDING on invalid padding
 *                      while decrypting, or
 *                      a cipher specific error code.
 */
int ingeek_cipher_crypt( ingeek_cipher_context_t *ctx,
                  const unsigned char *iv, size_t iv_len,
                  const unsigned char *input, size_t ilen,
                  unsigned char *output, size_t *olen );

#endif /* INGEEK_SMALL_CIPHER_C */


#if defined(INGEEK_SMALL_CIPHER_C)
#define CIPHER_PADDING_PKCS7
#define CIPHER_LEN    128
#define BLOCKLEN 16 
#endif


int ingeek_aes_ecb_encrypt(unsigned char *key,unsigned char*input,size_t inlen,unsigned char*output);

int ingeek_cipher_aes_cbc(unsigned char* key,unsigned char*iv,
							unsigned char*in,unsigned int ilen,
							unsigned char*out,unsigned int *olen,
							ingeek_operation_t mode);

#ifdef __cplusplus
}
#endif

#endif

