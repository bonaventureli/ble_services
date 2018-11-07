#if !defined(INGEEK_CONFIG_FILE)
#include "config.h"
#else
#include INGEEK_CONFIG_FILE
#endif

#if !defined(INGEEK_SMALL_CIPHER_C)


#if defined(INGEEK_ASN1_PARSE_C)

#include "asn1parse.h"

#include <string.h>

#if defined(INGEEK_BIGNUM_C)
#include "bignum.h"
#endif

#if defined(INGEEK_PLATFORM_C)
#include "platform.h"
#else
#include <stdlib.h>
#define ingeek_calloc    calloc
#define ingeek_free       free
#endif


#if defined(INGEEK_BIGNUM_C)
int ingeek_asn1_get_mpi( unsigned char **p,
                  const unsigned char *end,
                  ingeek_mpi *X )
{
    int ret;
    size_t len;

    if( ( ret = ingeek_asn1_get_tag( p, end, &len, INGEEK_ASN1_INTEGER ) ) != 0 )
        return( ret );

    ret = ingeek_mpi_read_binary( X, *p, len );

    *p += len;

    return( ret );
}
#endif /* INGEEK_BIGNUM_C */

/*
 * ASN.1 DER decoding routines
 */
int ingeek_asn1_get_len( unsigned char **p,
                  const unsigned char *end,
                  size_t *len )
{
    if( ( end - *p ) < 1 )
        return( INGEEK_ERR_ASN1_OUT_OF_DATA );

    if( ( **p & 0x80 ) == 0 )
        *len = *(*p)++;
    else
    {
        switch( **p & 0x7F )
        {
        case 1:
            if( ( end - *p ) < 2 )
                return( INGEEK_ERR_ASN1_OUT_OF_DATA );

            *len = (*p)[1];
            (*p) += 2;
            break;

        case 2:
            if( ( end - *p ) < 3 )
                return( INGEEK_ERR_ASN1_OUT_OF_DATA );

            *len = ( (size_t)(*p)[1] << 8 ) | (*p)[2];
            (*p) += 3;
            break;

        case 3:
            if( ( end - *p ) < 4 )
                return( INGEEK_ERR_ASN1_OUT_OF_DATA );

            *len = ( (size_t)(*p)[1] << 16 ) |
                   ( (size_t)(*p)[2] << 8  ) | (*p)[3];
            (*p) += 4;
            break;

        case 4:
            if( ( end - *p ) < 5 )
                return( INGEEK_ERR_ASN1_OUT_OF_DATA );

            *len = ( (size_t)(*p)[1] << 24 ) | ( (size_t)(*p)[2] << 16 ) |
                   ( (size_t)(*p)[3] << 8  ) |           (*p)[4];
            (*p) += 5;
            break;

        default:
            return( INGEEK_ERR_ASN1_INVALID_LENGTH );
        }
    }

    if( *len > (size_t) ( end - *p ) )
        return( INGEEK_ERR_ASN1_OUT_OF_DATA );

    return( 0 );
}

int ingeek_asn1_get_tag( unsigned char **p,
                  const unsigned char *end,
                  size_t *len, int tag )
{
    if( ( end - *p ) < 1 )
        return( INGEEK_ERR_ASN1_OUT_OF_DATA );

    if( **p != tag )
        return( INGEEK_ERR_ASN1_UNEXPECTED_TAG );

    (*p)++;

    return( ingeek_asn1_get_len( p, end, len ) );
}


#if defined(INGEEK_UNUSED_FUN)
int ingeek_asn1_get_bool( unsigned char **p,
                   const unsigned char *end,
                   int *val )
{
    int ret;
    size_t len;

    if( ( ret = ingeek_asn1_get_tag( p, end, &len, INGEEK_ASN1_BOOLEAN ) ) != 0 )
        return( ret );

    if( len != 1 )
        return( INGEEK_ERR_ASN1_INVALID_LENGTH );

    *val = ( **p != 0 ) ? 1 : 0;
    (*p)++;

    return( 0 );
}

int ingeek_asn1_get_int( unsigned char **p,
                  const unsigned char *end,
                  int *val )
{
    int ret;
    size_t len;

    if( ( ret = ingeek_asn1_get_tag( p, end, &len, INGEEK_ASN1_INTEGER ) ) != 0 )
        return( ret );

    if( len == 0 || len > sizeof( int ) || ( **p & 0x80 ) != 0 )
        return( INGEEK_ERR_ASN1_INVALID_LENGTH );

    *val = 0;

    while( len-- > 0 )
    {
        *val = ( *val << 8 ) | **p;
        (*p)++;
    }

    return( 0 );
}


int ingeek_asn1_get_bitstring( unsigned char **p, const unsigned char *end,
                        ingeek_asn1_bitstring *bs)
{
    int ret;

    /* Certificate type is a single byte bitstring */
    if( ( ret = ingeek_asn1_get_tag( p, end, &bs->len, INGEEK_ASN1_BIT_STRING ) ) != 0 )
        return( ret );

    /* Check length, subtract one for actual bit string length */
    if( bs->len < 1 )
        return( INGEEK_ERR_ASN1_OUT_OF_DATA );
    bs->len -= 1;

    /* Get number of unused bits, ensure unused bits <= 7 */
    bs->unused_bits = **p;
    if( bs->unused_bits > 7 )
        return( INGEEK_ERR_ASN1_INVALID_LENGTH );
    (*p)++;

    /* Get actual bitstring */
    bs->p = *p;
    *p += bs->len;

    if( *p != end )
        return( INGEEK_ERR_ASN1_LENGTH_MISMATCH );

    return( 0 );
}

/*
 * Get a bit string without unused bits
 */
int ingeek_asn1_get_bitstring_null( unsigned char **p, const unsigned char *end,
                             size_t *len )
{
    int ret;

    if( ( ret = ingeek_asn1_get_tag( p, end, len, INGEEK_ASN1_BIT_STRING ) ) != 0 )
        return( ret );

    if( (*len)-- < 2 || *(*p)++ != 0 )
        return( INGEEK_ERR_ASN1_INVALID_DATA );

    return( 0 );
}



/*
 *  Parses and splits an ASN.1 "SEQUENCE OF <tag>"
 */
int ingeek_asn1_get_sequence_of( unsigned char **p,
                          const unsigned char *end,
                          ingeek_asn1_sequence *cur,
                          int tag)
{
    int ret;
    size_t len;
    ingeek_asn1_buf *buf;

    /* Get main sequence tag */
    if( ( ret = ingeek_asn1_get_tag( p, end, &len,
            INGEEK_ASN1_CONSTRUCTED | INGEEK_ASN1_SEQUENCE ) ) != 0 )
        return( ret );

    if( *p + len != end )
        return( INGEEK_ERR_ASN1_LENGTH_MISMATCH );

    while( *p < end )
    {
        buf = &(cur->buf);
        buf->tag = **p;

        if( ( ret = ingeek_asn1_get_tag( p, end, &buf->len, tag ) ) != 0 )
            return( ret );

        buf->p = *p;
        *p += buf->len;

        /* Allocate and assign next pointer */
        if( *p < end )
        {
            cur->next = (ingeek_asn1_sequence*)ingeek_calloc( 1,
                                            sizeof( ingeek_asn1_sequence ) );

            if( cur->next == NULL )
                return( INGEEK_ERR_ASN1_ALLOC_FAILED );

            cur = cur->next;
        }
    }

    /* Set final sequence entry's next pointer to NULL */
    cur->next = NULL;

    if( *p != end )
        return( INGEEK_ERR_ASN1_LENGTH_MISMATCH );

    return( 0 );
}

int ingeek_asn1_get_alg( unsigned char **p,
                  const unsigned char *end,
                  ingeek_asn1_buf *alg, ingeek_asn1_buf *params )
{
    int ret;
    size_t len;

    if( ( ret = ingeek_asn1_get_tag( p, end, &len,
            INGEEK_ASN1_CONSTRUCTED | INGEEK_ASN1_SEQUENCE ) ) != 0 )
        return( ret );

    if( ( end - *p ) < 1 )
        return( INGEEK_ERR_ASN1_OUT_OF_DATA );

    alg->tag = **p;
    end = *p + len;

    if( ( ret = ingeek_asn1_get_tag( p, end, &alg->len, INGEEK_ASN1_OID ) ) != 0 )
        return( ret );

    alg->p = *p;
    *p += alg->len;

    if( *p == end )
    {
        ingeek_zeroize( params, sizeof(ingeek_asn1_buf) );
        return( 0 );
    }

    params->tag = **p;
    (*p)++;

    if( ( ret = ingeek_asn1_get_len( p, end, &params->len ) ) != 0 )
        return( ret );

    params->p = *p;
    *p += params->len;

    if( *p != end )
        return( INGEEK_ERR_ASN1_LENGTH_MISMATCH );

    return( 0 );
}

int ingeek_asn1_get_alg_null( unsigned char **p,
                       const unsigned char *end,
                       ingeek_asn1_buf *alg )
{
    int ret;
    ingeek_asn1_buf params;

    memset( &params, 0, sizeof(ingeek_asn1_buf) );

    if( ( ret = ingeek_asn1_get_alg( p, end, alg, &params ) ) != 0 )
        return( ret );

    if( ( params.tag != INGEEK_ASN1_NULL && params.tag != 0 ) || params.len != 0 )
        return( INGEEK_ERR_ASN1_INVALID_DATA );

    return( 0 );
}

void ingeek_asn1_free_named_data( ingeek_asn1_named_data *cur )
{
    if( cur == NULL )
        return;

    ingeek_free( cur->oid.p );
    ingeek_free( cur->val.p );

    ingeek_zeroize( cur, sizeof( ingeek_asn1_named_data ) );
}

void ingeek_asn1_free_named_data_list( ingeek_asn1_named_data **head )
{
    ingeek_asn1_named_data *cur;

    while( ( cur = *head ) != NULL )
    {
        *head = cur->next;
        ingeek_asn1_free_named_data( cur );
        ingeek_free( cur );
    }
}

ingeek_asn1_named_data *ingeek_asn1_find_named_data( ingeek_asn1_named_data *list,
                                       const char *oid, size_t len )
{
    while( list != NULL )
    {
        if( list->oid.len == len &&
            memcmp( list->oid.p, oid, len ) == 0 )
        {
            break;
        }

        list = list->next;
    }

    return( list );
}
#endif /*INGEEK_UNUSED_FUN*/
#endif /* INGEEK_ASN1_PARSE_C */
#endif
