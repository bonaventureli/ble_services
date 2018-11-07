#if !defined(INGEEK_CONFIG_FILE)
#include "config.h"
#else
#include INGEEK_CONFIG_FILE
#endif

#if defined(INGEEK_MD_C)

#include "md_wrap.h"

#if defined(INGEEK_SHA256_C)
#include "sha256.h"
#endif

#if defined(INGEEK_SHA512_C)
#include "sha512.h"
#endif

#if defined(INGEEK_PLATFORM_C)
#include "platform.h"
#else
#include <stdlib.h>
#define ingeek_calloc    calloc
#define ingeek_free       free
#endif

/*
 * Wrappers for generic message digests
 */
#if defined(INGEEK_SHA256_C)

static void sha224_starts_wrap( void *ctx )
{
    ingeek_sha256_starts( (ingeek_sha256_context *) ctx, 1 );
}

static void sha224_update_wrap( void *ctx, const unsigned char *input,
                                size_t ilen )
{
    ingeek_sha256_update( (ingeek_sha256_context *) ctx, input, ilen );
}

static void sha224_finish_wrap( void *ctx, unsigned char *output )
{
    ingeek_sha256_finish( (ingeek_sha256_context *) ctx, output );
}

static void sha224_wrap( const unsigned char *input, size_t ilen,
                    unsigned char *output )
{
    ingeek_sha256( input, ilen, output, 1 );
}

static void *sha224_ctx_alloc( void )
{
    void *ctx = ingeek_calloc( 1, sizeof(ingeek_sha256_context) );

    if( ctx != NULL )
        ingeek_sha256_init( (ingeek_sha256_context *) ctx );

    return( ctx );
}

static void sha224_ctx_free( void *ctx )
{
    ingeek_sha256_free( (ingeek_sha256_context *) ctx );
    ingeek_free( ctx );
}


const ingeek_md_info_t ingeek_sha224_info = {
    INGEEK_MD_SHA224,
    "SHA224",
    28,
    64,
    sha224_starts_wrap,
    sha224_update_wrap,
    sha224_finish_wrap,
    sha224_wrap,
    sha224_ctx_alloc,
    sha224_ctx_free,
};

static void sha256_starts_wrap( void *ctx )
{
    ingeek_sha256_starts( (ingeek_sha256_context *) ctx, 0 );
}

static void sha256_wrap( const unsigned char *input, size_t ilen,
                    unsigned char *output )
{
    ingeek_sha256( input, ilen, output, 0 );
}

const ingeek_md_info_t ingeek_sha256_info = {
    INGEEK_MD_SHA256,
    "SHA256",
    32,
    64,
    sha256_starts_wrap,
    sha224_update_wrap,
    sha224_finish_wrap,
    sha256_wrap,
    sha224_ctx_alloc,
    sha224_ctx_free,
};

#endif /* INGEEK_SHA256_C */

#if defined(INGEEK_SHA512_C)

static void sha384_starts_wrap( void *ctx )
{
    ingeek_sha512_starts( (ingeek_sha512_context *) ctx, 1 );
}

static void sha384_update_wrap( void *ctx, const unsigned char *input,
                                size_t ilen )
{
    ingeek_sha512_update( (ingeek_sha512_context *) ctx, input, ilen );
}

static void sha384_finish_wrap( void *ctx, unsigned char *output )
{
    ingeek_sha512_finish( (ingeek_sha512_context *) ctx, output );
}

static void sha384_wrap( const unsigned char *input, size_t ilen,
                    unsigned char *output )
{
    ingeek_sha512( input, ilen, output, 1 );
}

static void *sha384_ctx_alloc( void )
{
    void *ctx = ingeek_calloc( 1, sizeof( ingeek_sha512_context ) );

    if( ctx != NULL )
        ingeek_sha512_init( (ingeek_sha512_context *) ctx );

    return( ctx );
}

static void sha384_ctx_free( void *ctx )
{
    ingeek_sha512_free( (ingeek_sha512_context *) ctx );
    ingeek_free( ctx );
}


const ingeek_md_info_t ingeek_sha384_info = {
    INGEEK_MD_SHA384,
    "SHA384",
    48,
    128,
    sha384_starts_wrap,
    sha384_update_wrap,
    sha384_finish_wrap,
    sha384_wrap,
    sha384_ctx_alloc,
    sha384_ctx_free,
};

static void sha512_starts_wrap( void *ctx )
{
    ingeek_sha512_starts( (ingeek_sha512_context *) ctx, 0 );
}

static void sha512_wrap( const unsigned char *input, size_t ilen,
                    unsigned char *output )
{
    ingeek_sha512( input, ilen, output, 0 );
}

const ingeek_md_info_t ingeek_sha512_info = {
    INGEEK_MD_SHA512,
    "SHA512",
    64,
    128,
    sha512_starts_wrap,
    sha384_update_wrap,
    sha384_finish_wrap,
    sha512_wrap,
    sha384_ctx_alloc,
    sha384_ctx_free,
};

#endif /* INGEEK_SHA512_C */

#endif /* INGEEK_MD_C */
