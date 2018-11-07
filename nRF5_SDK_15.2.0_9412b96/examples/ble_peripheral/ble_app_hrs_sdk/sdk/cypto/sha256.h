#ifndef INGEEK_SHA256_H
#define INGEEK_SHA256_H

#include <stddef.h>
//#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif



/**
 * \brief          SHA-256 context structure
 */
typedef struct
{
    uint32_t total[2];          /*!< number of bytes processed  */
    uint32_t state[8];          /*!< intermediate digest state  */
    unsigned char buffer[64];   /*!< data block being processed */
    int is224;                  /*!< 0 => SHA-256, else SHA-224 */
}ingeek_sha256_context;


/**
 * \brief          Initialize SHA-256 context
 *
 * \param ctx      SHA-256 context to be initialized
 */
void ingeek_sha256_init( ingeek_sha256_context *ctx );

/**
 * \brief          Clear SHA-256 context
 *
 * \param ctx      SHA-256 context to be cleared
 */
void ingeek_sha256_free( ingeek_sha256_context *ctx );


/**
 * \brief          SHA-256 context setup
 *
 * \param ctx      context to be initialized
 * \param is224    0 = use SHA256, 1 = use SHA224
 */
void ingeek_sha256_starts( ingeek_sha256_context *ctx, int is224 );

/**
 * \brief          SHA-256 process buffer
 *
 * \param ctx      SHA-256 context
 * \param input    buffer holding the  data
 * \param ilen     length of the input data
 */
void ingeek_sha256_update( ingeek_sha256_context *ctx, const unsigned char *input,
                    size_t ilen );

/**
 * \brief          SHA-256 final digest
 *
 * \param ctx      SHA-256 context
 * \param output   SHA-224/256 checksum result
 */
void ingeek_sha256_finish( ingeek_sha256_context *ctx, unsigned char output[32] );

/**
 * \brief          Output = SHA-256( input buffer )
 *
 * \param input    buffer holding the  data
 * \param ilen     length of the input data
 * \param output   SHA-224/256 checksum result
 * \param is224    0 = use SHA256, 1 = use SHA224
 */
void ingeek_sha256( const unsigned char *input, size_t ilen,
           unsigned char output[32], int is224 );


#ifdef __cplusplus
}
#endif

#endif /* ingeek_sha256.h */
