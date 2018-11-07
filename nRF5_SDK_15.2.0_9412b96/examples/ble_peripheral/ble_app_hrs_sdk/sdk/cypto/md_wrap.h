#ifndef INGEEK_MD_WRAP_H
#define INGEEK_MD_WRAP_H

#if !defined(INGEEK_CONFIG_FILE)
#include "config.h"
#else
#include INGEEK_CONFIG_FILE
#endif

#include "md_export.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Message digest information.
 * Allows message digest functions to be called in a generic way.
 */
struct ingeek_md_info_t
{
    /** Digest identifier */
    ingeek_md_type_t type;

    /** Name of the message digest */
    const char * name;

    /** Output length of the digest function in bytes */
    int size;

    /** Block length of the digest function in bytes */
    int block_size;

    /** Digest initialisation function */
    void (*starts_func)( void *ctx );

    /** Digest update function */
    void (*update_func)( void *ctx, const unsigned char *input, size_t ilen );

    /** Digest finalisation function */
    void (*finish_func)( void *ctx, unsigned char *output );

    /** Generic digest function */
    void (*digest_func)( const unsigned char *input, size_t ilen,
                         unsigned char *output );

    /** Allocate a new context */
    void * (*ctx_alloc_func)( void );

    /** Free the given context */
    void (*ctx_free_func)( void *ctx );
};

#if defined(INGEEK_SHA256_C)
extern const ingeek_md_info_t ingeek_sha224_info;
extern const ingeek_md_info_t ingeek_sha256_info;
#endif
#if defined(INGEEK_SHA512_C)
extern const ingeek_md_info_t ingeek_sha384_info;
extern const ingeek_md_info_t ingeek_sha512_info;
#endif

#ifdef __cplusplus
}
#endif

#endif /* INGEEK_MD_WRAP_H */
