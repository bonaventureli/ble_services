#ifndef INGEEK_MD_H
#define INGEEK_MD_H


#include <stddef.h>
//#include <stdint.h>

#define INGEEK_ERR_MD_FEATURE_UNAVAILABLE                -0x5080  /**< The selected feature is not available. */
#define INGEEK_ERR_MD_BAD_INPUT_DATA                     -0x5100  /**< Bad input parameters to function. */
#define INGEEK_ERR_MD_ALLOC_FAILED                       -0x5180  /**< Failed to allocate memory. */
#define INGEEK_ERR_MD_FILE_IO_ERROR                      -0x5200  /**< Opening or reading of file failed. */

#ifdef __cplusplus
extern "C" {
#endif


typedef enum {
    INGEEK_MD_NONE=0,
    INGEEK_MD_SHA224,
    INGEEK_MD_SHA256,
    INGEEK_MD_SHA384,
    INGEEK_MD_SHA512,
} ingeek_md_type_t;

#if defined(INGEEK_SHA512_C)
#define INGEEK_MD_MAX_SIZE         64
#else
#define INGEEK_MD_MAX_SIZE         32
#endif

/**
 * Opaque struct defined in md_internal.h
 */
typedef struct ingeek_md_info_t ingeek_md_info_t;

/**
 * Generic message digest context.
 */
typedef struct {
    const ingeek_md_info_t *md_info;
    void *md_ctx;
    void *hmac_ctx;
} ingeek_md_context_t;


/**
 * \brief           Returns the message digest information associated with the
 *                  given digest type.
 *
 * \param md_type   type of digest to search for.
 *
 * \return          The message digest information associated with md_type or
 *                  NULL if not found.
 */
const ingeek_md_info_t *Get_md_info_from_type( ingeek_md_type_t md_type );

/**
 * \brief          Output = message_digest( input buffer )
 *
 * \param md_info  message digest info
 * \param input    buffer holding the  data
 * \param ilen     length of the input data
 * \param output   Generic message digest checksum result
 *
 * \returns        0 on success, INGEEK_ERR_MD_BAD_INPUT_DATA if parameter
 *                 verification fails.
 */
int ingeek_message_digest( const ingeek_md_info_t *md_info, 
					const unsigned char *input, size_t ilen,
       				 unsigned char *output );

/**
 * \brief          Output = Generic_HMAC( hmac key, input buffer )
 *
 * \param md_info  message digest info
 * \param key      HMAC secret key
 * \param keylen   length of the HMAC key in bytes
 * \param input    buffer holding the  data
 * \param ilen     length of the input data
 * \param output   Generic HMAC-result
 *
 * \returns        0 on success, INGEEK_ERR_MD_BAD_INPUT_DATA if parameter
 *                 verification fails.
 */
int ingeek_message_digest_hmac( const ingeek_md_info_t *md_info, 
				const unsigned char *key, size_t keylen,
                const unsigned char *input, size_t ilen,
                unsigned char *output );

#ifdef __cplusplus
}
#endif

#endif /* INGEEK_MD_H */
