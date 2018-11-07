#if !defined(INGEEK_CONFIG_FILE)
#include "config.h"
#else
#include INGEEK_CONFIG_FILE
#endif

#if defined(INGEEK_MD_C)

#include "md_export.h"
#include "md_wrap.h"

#if defined(INGEEK_PLATFORM_C)
#include "platform.h"
#else
#include <stdlib.h>
#define ingeek_calloc    calloc
#define ingeek_free       free
#endif

#include <string.h>

/* Implementation that should never be optimized out by the compiler */
static void ingeek_zeroize( void *v, size_t n ) {
    volatile unsigned char *p = v; while( n-- ) *p++ = 0;
}


const ingeek_md_info_t *Get_md_info_from_type( ingeek_md_type_t md_type )
{
    switch( md_type )
    {
#if defined(INGEEK_SHA256_C)
        case INGEEK_MD_SHA224:
            return( &ingeek_sha224_info );
        case INGEEK_MD_SHA256:
            return( &ingeek_sha256_info );
#endif
#if defined(INGEEK_SHA512_C)
        case INGEEK_MD_SHA384:
            return( &ingeek_sha384_info );
        case INGEEK_MD_SHA512:
            return( &ingeek_sha512_info );
#endif
        default:
            return( NULL );
    }
}

void ingeek_md_init( ingeek_md_context_t *ctx )
{
    memset( ctx, 0, sizeof( ingeek_md_context_t ) );
}

void ingeek_md_free( ingeek_md_context_t *ctx )
{
    if( ctx == NULL || ctx->md_info == NULL )
        return;

    if( ctx->md_ctx != NULL )
        ctx->md_info->ctx_free_func( ctx->md_ctx );

    if( ctx->hmac_ctx != NULL )
    {
        ingeek_zeroize( ctx->hmac_ctx, 2 * ctx->md_info->block_size );
        ingeek_free( ctx->hmac_ctx );
    }

    ingeek_zeroize( ctx, sizeof( ingeek_md_context_t ) );
}


int ingeek_md_setup( ingeek_md_context_t *ctx, const ingeek_md_info_t *md_info, int hmac )
{
    if( md_info == NULL || ctx == NULL )
        return( INGEEK_ERR_MD_BAD_INPUT_DATA );

    if( ( ctx->md_ctx = md_info->ctx_alloc_func() ) == NULL )
        return( INGEEK_ERR_MD_ALLOC_FAILED );

    if( hmac != 0 )
    {
        ctx->hmac_ctx = ingeek_calloc( 2, md_info->block_size );
        if( ctx->hmac_ctx == NULL )
        {
            md_info->ctx_free_func( ctx->md_ctx );
            return( INGEEK_ERR_MD_ALLOC_FAILED );
        }
    }

    ctx->md_info = md_info;

    return( 0 );
}

int ingeek_md_starts( ingeek_md_context_t *ctx )
{
    if( ctx == NULL || ctx->md_info == NULL )
        return( INGEEK_ERR_MD_BAD_INPUT_DATA );

    ctx->md_info->starts_func( ctx->md_ctx );

    return( 0 );
}

int ingeek_md_update( ingeek_md_context_t *ctx, const unsigned char *input, size_t ilen )
{
    if( ctx == NULL || ctx->md_info == NULL )
        return( INGEEK_ERR_MD_BAD_INPUT_DATA );

    ctx->md_info->update_func( ctx->md_ctx, input, ilen );

    return( 0 );
}

int ingeek_md_finish( ingeek_md_context_t *ctx, unsigned char *output )
{
    if( ctx == NULL || ctx->md_info == NULL )
        return( INGEEK_ERR_MD_BAD_INPUT_DATA );

    ctx->md_info->finish_func( ctx->md_ctx, output );

    return( 0 );
}


int ingeek_md_hmac_starts( ingeek_md_context_t *ctx, const unsigned char *key, size_t keylen )
{
    unsigned char sum[INGEEK_MD_MAX_SIZE];
    unsigned char *ipad, *opad;
    size_t i;

    if( ctx == NULL || ctx->md_info == NULL || ctx->hmac_ctx == NULL )
        return( INGEEK_ERR_MD_BAD_INPUT_DATA );

    if( keylen > (size_t) ctx->md_info->block_size )
    {
        ctx->md_info->starts_func( ctx->md_ctx );
        ctx->md_info->update_func( ctx->md_ctx, key, keylen );
        ctx->md_info->finish_func( ctx->md_ctx, sum );

        keylen = ctx->md_info->size;
        key = sum;
    }

    ipad = (unsigned char *) ctx->hmac_ctx;
    opad = (unsigned char *) ctx->hmac_ctx + ctx->md_info->block_size;

    memset( ipad, 0x36, ctx->md_info->block_size );
    memset( opad, 0x5C, ctx->md_info->block_size );

    for( i = 0; i < keylen; i++ )
    {
        ipad[i] = (unsigned char)( ipad[i] ^ key[i] );
        opad[i] = (unsigned char)( opad[i] ^ key[i] );
    }

    ingeek_zeroize( sum, sizeof( sum ) );

    ctx->md_info->starts_func( ctx->md_ctx );
    ctx->md_info->update_func( ctx->md_ctx, ipad, ctx->md_info->block_size );

    return( 0 );
}

int ingeek_md_hmac_update( ingeek_md_context_t *ctx, const unsigned char *input, size_t ilen )
{
    if( ctx == NULL || ctx->md_info == NULL || ctx->hmac_ctx == NULL )
        return( INGEEK_ERR_MD_BAD_INPUT_DATA );

    ctx->md_info->update_func( ctx->md_ctx, input, ilen );

    return( 0 );
}

int ingeek_md_hmac_finish( ingeek_md_context_t *ctx, unsigned char *output )
{
    unsigned char tmp[INGEEK_MD_MAX_SIZE];
    unsigned char *opad;

    if( ctx == NULL || ctx->md_info == NULL || ctx->hmac_ctx == NULL )
        return( INGEEK_ERR_MD_BAD_INPUT_DATA );

    opad = (unsigned char *) ctx->hmac_ctx + ctx->md_info->block_size;

    ctx->md_info->finish_func( ctx->md_ctx, tmp );
    ctx->md_info->starts_func( ctx->md_ctx );
    ctx->md_info->update_func( ctx->md_ctx, opad, ctx->md_info->block_size );
    ctx->md_info->update_func( ctx->md_ctx, tmp, ctx->md_info->size );
    ctx->md_info->finish_func( ctx->md_ctx, output );

    return( 0 );
}

int ingeek_message_digest( const ingeek_md_info_t *md_info, const unsigned char *input, size_t ilen,
            unsigned char *output )
{
    if( md_info == NULL )
        return( INGEEK_ERR_MD_BAD_INPUT_DATA );

    md_info->digest_func( input, ilen, output );

    return( 0 );
}

int ingeek_message_digest_hmac( const ingeek_md_info_t *md_info, const unsigned char *key, size_t keylen,
                const unsigned char *input, size_t ilen,
                unsigned char *output )
{
    ingeek_md_context_t ctx;
    int ret;

    if( md_info == NULL )
        return( INGEEK_ERR_MD_BAD_INPUT_DATA );

    ingeek_md_init( &ctx );

    if( ( ret = ingeek_md_setup( &ctx, md_info, 1 ) ) != 0 )
        return( ret );

    ingeek_md_hmac_starts( &ctx, key, keylen );
    ingeek_md_hmac_update( &ctx, input, ilen );
    ingeek_md_hmac_finish( &ctx, output );

    ingeek_md_free( &ctx );

    return( 0 );
}

#endif /* INGEEK_MD_C */
