#if !defined(INGEEK_CONFIG_FILE)
#include "config.h"
#else
#include INGEEK_CONFIG_FILE
#endif

#if !defined(INGEEK_SMALL_CIPHER_C)

#if defined(INGEEK_ASN1_WRITE_C)

#include "asn1write.h"

#include <string.h>

#if defined(INGEEK_PLATFORM_C)
#include "platform.h"
#else
#include <stdlib.h>
#define ingeek_calloc    calloc
#define ingeek_free       free
#endif

#if defined(INGEEK_BIGNUM_C)
int ingeek_asn1_write_mpi( unsigned char **p, unsigned char *start, const ingeek_mpi *X )
{
    int ret;
    size_t len = 0;

    // Write the MPI
    //
    len = ingeek_mpi_size( X );

    if( *p < start || (size_t)( *p - start ) < len )
        return( INGEEK_ERR_ASN1_BUF_TOO_SMALL );

    (*p) -= len;
    INGEEK_MPI_CHK( ingeek_mpi_write_binary( X, *p, len ) );

    // DER format assumes 2s complement for numbers, so the leftmost bit
    // should be 0 for positive numbers and 1 for negative numbers.
    //
    if( X->s ==1 && **p & 0x80 )
    {
        if( *p - start < 1 )
            return( INGEEK_ERR_ASN1_BUF_TOO_SMALL );

        *--(*p) = 0x00;
        len += 1;
    }

    INGEEK_ASN1_CHK_ADD( len, ingeek_asn1_write_len( p, start, len ) );
    INGEEK_ASN1_CHK_ADD( len, ingeek_asn1_write_tag( p, start, INGEEK_ASN1_INTEGER ) );

    ret = (int) len;

cleanup:
    return( ret );
}
#endif /* INGEEK_BIGNUM_C */

int ingeek_asn1_write_len( unsigned char **p, unsigned char *start, size_t len )
{
    if( len < 0x80 )
    {
        if( *p - start < 1 )
            return( INGEEK_ERR_ASN1_BUF_TOO_SMALL );

        *--(*p) = (unsigned char) len;
        return( 1 );
    }

    if( len <= 0xFF )
    {
        if( *p - start < 2 )
            return( INGEEK_ERR_ASN1_BUF_TOO_SMALL );

        *--(*p) = (unsigned char) len;
        *--(*p) = 0x81;
        return( 2 );
    }

    if( len <= 0xFFFF )
    {
        if( *p - start < 3 )
            return( INGEEK_ERR_ASN1_BUF_TOO_SMALL );

        *--(*p) = ( len       ) & 0xFF;
        *--(*p) = ( len >>  8 ) & 0xFF;
        *--(*p) = 0x82;
        return( 3 );
    }

    if( len <= 0xFFFFFF )
    {
        if( *p - start < 4 )
            return( INGEEK_ERR_ASN1_BUF_TOO_SMALL );

        *--(*p) = ( len       ) & 0xFF;
        *--(*p) = ( len >>  8 ) & 0xFF;
        *--(*p) = ( len >> 16 ) & 0xFF;
        *--(*p) = 0x83;
        return( 4 );
    }

    if( len <= 0xFFFFFFFF )
    {
        if( *p - start < 5 )
            return( INGEEK_ERR_ASN1_BUF_TOO_SMALL );

        *--(*p) = ( len       ) & 0xFF;
        *--(*p) = ( len >>  8 ) & 0xFF;
        *--(*p) = ( len >> 16 ) & 0xFF;
        *--(*p) = ( len >> 24 ) & 0xFF;
        *--(*p) = 0x84;
        return( 5 );
    }

    return( INGEEK_ERR_ASN1_INVALID_LENGTH );
}

int ingeek_asn1_write_tag( unsigned char **p, unsigned char *start, unsigned char tag )
{
    if( *p - start < 1 )
        return( INGEEK_ERR_ASN1_BUF_TOO_SMALL );

    *--(*p) = tag;

    return( 1 );
}

#if defined(INGEEK_UNUSED_FUN)
int ingeek_asn1_write_raw_buffer( unsigned char **p, unsigned char *start,
                           const unsigned char *buf, size_t size )
{
    size_t len = 0;

    if( *p < start || (size_t)( *p - start ) < size )
        return( INGEEK_ERR_ASN1_BUF_TOO_SMALL );

    len = size;
    (*p) -= len;
    memcpy( *p, buf, len );

    return( (int) len );
}

int ingeek_asn1_write_null( unsigned char **p, unsigned char *start )
{
    int ret;
    size_t len = 0;

    // Write NULL
    //
    INGEEK_ASN1_CHK_ADD( len, ingeek_asn1_write_len( p, start, 0) );
    INGEEK_ASN1_CHK_ADD( len, ingeek_asn1_write_tag( p, start, INGEEK_ASN1_NULL ) );

    return( (int) len );
}

int ingeek_asn1_write_oid( unsigned char **p, unsigned char *start,
                    const char *oid, size_t oid_len )
{
    int ret;
    size_t len = 0;

    INGEEK_ASN1_CHK_ADD( len, ingeek_asn1_write_raw_buffer( p, start,
                                  (const unsigned char *) oid, oid_len ) );
    INGEEK_ASN1_CHK_ADD( len , ingeek_asn1_write_len( p, start, len ) );
    INGEEK_ASN1_CHK_ADD( len , ingeek_asn1_write_tag( p, start, INGEEK_ASN1_OID ) );

    return( (int) len );
}

int ingeek_asn1_write_algorithm_identifier( unsigned char **p, unsigned char *start,
                                     const char *oid, size_t oid_len,
                                     size_t par_len )
{
    int ret;
    size_t len = 0;

    if( par_len == 0 )
        INGEEK_ASN1_CHK_ADD( len, ingeek_asn1_write_null( p, start ) );
    else
        len += par_len;

    INGEEK_ASN1_CHK_ADD( len, ingeek_asn1_write_oid( p, start, oid, oid_len ) );

    INGEEK_ASN1_CHK_ADD( len, ingeek_asn1_write_len( p, start, len ) );
    INGEEK_ASN1_CHK_ADD( len, ingeek_asn1_write_tag( p, start,
                                       INGEEK_ASN1_CONSTRUCTED | INGEEK_ASN1_SEQUENCE ) );

    return( (int) len );
}

int ingeek_asn1_write_bool( unsigned char **p, unsigned char *start, int boolean )
{
    int ret;
    size_t len = 0;

    if( *p - start < 1 )
        return( INGEEK_ERR_ASN1_BUF_TOO_SMALL );

    *--(*p) = (boolean) ? 255 : 0;
    len++;

    INGEEK_ASN1_CHK_ADD( len, ingeek_asn1_write_len( p, start, len ) );
    INGEEK_ASN1_CHK_ADD( len, ingeek_asn1_write_tag( p, start, INGEEK_ASN1_BOOLEAN ) );

    return( (int) len );
}

int ingeek_asn1_write_int( unsigned char **p, unsigned char *start, int val )
{
    int ret;
    size_t len = 0;

    // TODO negative values and values larger than 128
    // DER format assumes 2s complement for numbers, so the leftmost bit
    // should be 0 for positive numbers and 1 for negative numbers.
    //
    if( *p - start < 1 )
        return( INGEEK_ERR_ASN1_BUF_TOO_SMALL );

    len += 1;
    *--(*p) = val;

    if( val > 0 && **p & 0x80 )
    {
        if( *p - start < 1 )
            return( INGEEK_ERR_ASN1_BUF_TOO_SMALL );

        *--(*p) = 0x00;
        len += 1;
    }

    INGEEK_ASN1_CHK_ADD( len, ingeek_asn1_write_len( p, start, len ) );
    INGEEK_ASN1_CHK_ADD( len, ingeek_asn1_write_tag( p, start, INGEEK_ASN1_INTEGER ) );

    return( (int) len );
}

int ingeek_asn1_write_printable_string( unsigned char **p, unsigned char *start,
                                 const char *text, size_t text_len )
{
    int ret;
    size_t len = 0;

    INGEEK_ASN1_CHK_ADD( len, ingeek_asn1_write_raw_buffer( p, start,
                  (const unsigned char *) text, text_len ) );

    INGEEK_ASN1_CHK_ADD( len, ingeek_asn1_write_len( p, start, len ) );
    INGEEK_ASN1_CHK_ADD( len, ingeek_asn1_write_tag( p, start, INGEEK_ASN1_PRINTABLE_STRING ) );

    return( (int) len );
}

int ingeek_asn1_write_ia5_string( unsigned char **p, unsigned char *start,
                           const char *text, size_t text_len )
{
    int ret;
    size_t len = 0;

    INGEEK_ASN1_CHK_ADD( len, ingeek_asn1_write_raw_buffer( p, start,
                  (const unsigned char *) text, text_len ) );

    INGEEK_ASN1_CHK_ADD( len, ingeek_asn1_write_len( p, start, len ) );
    INGEEK_ASN1_CHK_ADD( len, ingeek_asn1_write_tag( p, start, INGEEK_ASN1_IA5_STRING ) );

    return( (int) len );
}

int ingeek_asn1_write_bitstring( unsigned char **p, unsigned char *start,
                          const unsigned char *buf, size_t bits )
{
    int ret;
    size_t len = 0, size;

    size = ( bits / 8 ) + ( ( bits % 8 ) ? 1 : 0 );

    // Calculate byte length
    //
    if( *p < start || (size_t)( *p - start ) < size + 1 )
        return( INGEEK_ERR_ASN1_BUF_TOO_SMALL );

    len = size + 1;
    (*p) -= size;
    memcpy( *p, buf, size );

    // Write unused bits
    //
    *--(*p) = (unsigned char) (size * 8 - bits);

    INGEEK_ASN1_CHK_ADD( len, ingeek_asn1_write_len( p, start, len ) );
    INGEEK_ASN1_CHK_ADD( len, ingeek_asn1_write_tag( p, start, INGEEK_ASN1_BIT_STRING ) );

    return( (int) len );
}

int ingeek_asn1_write_octet_string( unsigned char **p, unsigned char *start,
                             const unsigned char *buf, size_t size )
{
    int ret;
    size_t len = 0;

    INGEEK_ASN1_CHK_ADD( len, ingeek_asn1_write_raw_buffer( p, start, buf, size ) );

    INGEEK_ASN1_CHK_ADD( len, ingeek_asn1_write_len( p, start, len ) );
    INGEEK_ASN1_CHK_ADD( len, ingeek_asn1_write_tag( p, start, INGEEK_ASN1_OCTET_STRING ) );

    return( (int) len );
}

ingeek_asn1_named_data *ingeek_asn1_store_named_data( ingeek_asn1_named_data **head,
                                        const char *oid, size_t oid_len,
                                        const unsigned char *val,
                                        size_t val_len )
{
    ingeek_asn1_named_data *cur;

    if( ( cur = ingeek_asn1_find_named_data( *head, oid, oid_len ) ) == NULL )
    {
        // Add new entry if not present yet based on OID
        //
        cur = (ingeek_asn1_named_data*)ingeek_calloc( 1,
                                            sizeof(ingeek_asn1_named_data) );
        if( cur == NULL )
            return( NULL );

        cur->oid.len = oid_len;
        cur->oid.p = ingeek_calloc( 1, oid_len );
        if( cur->oid.p == NULL )
        {
            ingeek_free( cur );
            return( NULL );
        }

        memcpy( cur->oid.p, oid, oid_len );

        cur->val.len = val_len;
        cur->val.p = ingeek_calloc( 1, val_len );
        if( cur->val.p == NULL )
        {
            ingeek_free( cur->oid.p );
            ingeek_free( cur );
            return( NULL );
        }

        cur->next = *head;
        *head = cur;
    }
    else if( cur->val.len < val_len )
    {
        /*
         * Enlarge existing value buffer if needed
         * Preserve old data until the allocation succeeded, to leave list in
         * a consistent state in case allocation fails.
         */
        void *p = ingeek_calloc( 1, val_len );
        if( p == NULL )
            return( NULL );

        ingeek_free( cur->val.p );
        cur->val.p = p;
        cur->val.len = val_len;
    }

    if( val != NULL )
        memcpy( cur->val.p, val, val_len );

    return( cur );
}
#endif /*INGEEK_UNUSED_FUN*/
#endif /* INGEEK_ASN1_WRITE_C */
#endif /*#if !defined(INGEEK_SMALL_CIPHER_C)*/
