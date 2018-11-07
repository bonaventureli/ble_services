#ifndef INGEEK_AES_H
#define INGEEK_AES_H

#if !defined(INGEEK_CONFIG_FILE)
#include "config.h"
#else
#include INGEEK_CONFIG_FILE
#endif

#if !defined(INGEEK_SMALL_CIPHER_C)

#include <stddef.h>
//#include <stdint.h>

typedef enum {
    INGEEK_AES_DECRYPT = 0,
    INGEEK_AES_ENCRYPT,
} ingeek_mode_t;


#define INGEEK_ERR_AES_INVALID_KEY_LENGTH                -0x0020  /**< Invalid key length. */
#define INGEEK_ERR_AES_INVALID_INPUT_LENGTH              -0x0022  /**< Invalid data input length. */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief          AES context structure
 */
typedef struct
{
    int nr;                     /*!<  number of rounds  */
    uint32_t *rk;               /*!<  AES round keys    */
    uint32_t buf[68];           /*!<  unaligned data    */
}ingeek_aes_context;

/**
 * \brief          Initialize AES context
 *
 * \param ctx      AES context to be initialized
 */
void ingeek_aes_init( ingeek_aes_context *ctx );

/**
 * \brief          Clear AES context
 *
 * \param ctx      AES context to be cleared
 */
void ingeek_aes_free( ingeek_aes_context *ctx );

/**
 * \brief          AES key schedule (encryption)
 *
 * \param ctx      AES context to be initialized
 * \param key      encryption key
 * \param keybits  must be 128, 192 or 256
 *
 * \return         0 if successful, or INGEEK_ERR_AES_INVALID_KEY_LENGTH
 */
int ingeek_aes_setkey_enc( ingeek_aes_context *ctx, const unsigned char *key,
                    unsigned int keybits );

/**
 * \brief          AES key schedule (decryption)
 *
 * \param ctx      AES context to be initialized
 * \param key      decryption key
 * \param keybits  must be 128, 192 or 256
 *
 * \return         0 if successful, or INGEEK_ERR_AES_INVALID_KEY_LENGTH
 */
int ingeek_aes_setkey_dec( ingeek_aes_context *ctx, const unsigned char *key,
                    unsigned int keybits );

/**
 * \brief          AES-ECB block encryption/decryption
 *
 * \param ctx      AES context
 * \param mode     INGEEK_AES_ENCRYPT or INGEEK_AES_DECRYPT
 * \param input    16-byte input block
 * \param output   16-byte output block
 *
 * \return         0 if successful
 */
int ingeek_aes_crypt_ecb( ingeek_aes_context *ctx,
                    int mode,
                    const unsigned char input[16],
                    unsigned char output[16] );

/**
 * \brief          AES-CBC buffer encryption/decryption
 *                 Length should be a multiple of the block
 *                 size (16 bytes)
 *
 * \note           Upon exit, the content of the IV is updated so that you can
 *                 call the function same function again on the following
 *                 block(s) of data and get the same result as if it was
 *                 encrypted in one call. This allows a "streaming" usage.
 *                 If on the other hand you need to retain the contents of the
 *                 IV, you should either save it manually or use the cipher
 *                 module instead.
 *
 * \param ctx      AES context
 * \param mode     INGEEK_AES_ENCRYPT or INGEEK_AES_DECRYPT
 * \param length   length of the input data
 * \param iv       initialization vector (updated after use)
 * \param input    buffer holding the input data
 * \param output   buffer holding the output data
 *
 * \return         0 if successful, or INGEEK_ERR_AES_INVALID_INPUT_LENGTH
 */
 
#if defined(INGEEK_CIPHER_MODE_CBC)
int ingeek_aes_crypt_cbc( ingeek_aes_context *ctx,
                    int mode,
                    size_t length,
                    unsigned char iv[16],
                    const unsigned char *input,
                    unsigned char *output );
#endif /* INGEEK_CIPHER_MODE_CBC */

#else

#ifndef CBC
  #define CBC 1
#endif

#ifndef ECB
  #define ECB 1
#endif

#define AES128 1
//#define AES192 1
//#define AES256 1


#if defined(ECB) && (ECB == 1)
void AES_ECB_encrypt(const unsigned char* input, const unsigned char* key, unsigned char *output, const unsigned int length);
void AES_ECB_decrypt(const unsigned char* input, const unsigned char* key, unsigned char *output, const unsigned int length);
#endif // #if defined(ECB) && (ECB == !)

#if defined(CBC) && (CBC == 1)
void AES_CBC_encrypt_buffer(unsigned char* output, unsigned char* input, unsigned int length, const unsigned char* key, const unsigned char* iv);
void AES_CBC_decrypt_buffer(unsigned char* output, unsigned char* input, unsigned int length, const unsigned char* key, const unsigned char* iv);
#endif // #if defined(CBC) && (CBC == 1)

#endif /*INGEEK_SMALL_CIPHER_C*/

#ifdef __cplusplus
}
#endif

#endif /* aes.h */
