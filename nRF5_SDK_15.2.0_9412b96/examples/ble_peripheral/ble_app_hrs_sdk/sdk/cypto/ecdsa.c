#if !defined(INGEEK_CONFIG_FILE)
#include "config.h"
#else
#include INGEEK_CONFIG_FILE
#endif
#if !defined(INGEEK_SMALL_CIPHER_C)

#if defined(INGEEK_ECDSA_C)

#include "ecdsa.h"
#include "asn1write.h"
#include <string.h>


/*
 * Derive a suitable integer for group grp from a buffer of length len
 * SEC1 4.1.3 step 5 aka SEC1 4.1.4 step 3
 */
static int derive_mpi( const ingeek_ecp_group *grp, ingeek_mpi *x,
                       const unsigned char *buf, size_t blen )
{
    int ret;
    size_t n_size = ( grp->nbits + 7 ) / 8;
    size_t use_size = blen > n_size ? n_size : blen;

    INGEEK_MPI_CHK( ingeek_mpi_read_binary( x, buf, use_size ) );
    if( use_size * 8 > grp->nbits )
        INGEEK_MPI_CHK( ingeek_mpi_shift_r( x, use_size * 8 - grp->nbits ) );

    /* While at it, reduce modulo N */
    if( ingeek_mpi_cmp_mpi( x, &grp->N ) >= 0 )
        INGEEK_MPI_CHK( ingeek_mpi_sub_mpi( x, x, &grp->N ) );

cleanup:
    return( ret );
}

/*
 * Compute ECDSA signature of a hashed message (SEC1 4.1.3)
 * Obviously, compared to SEC1 4.1.3, we skip step 4 (hash message)
 */
int ingeek_ecdsa_sign( ingeek_ecp_group *grp, ingeek_mpi *r, ingeek_mpi *s,
                const ingeek_mpi *d, const unsigned char *buf, size_t blen,
                int (*f_rng)(void *, unsigned char *, size_t), void *p_rng )
{
    int ret, key_tries, sign_tries, blind_tries;
    ingeek_ecp_point R;
    ingeek_mpi k, e, t;

    /* Fail cleanly on curves such as Curve25519 that can't be used for ECDSA */
    if( grp->N.p == NULL )
        return( INGEEK_ERR_ECP_BAD_INPUT_DATA );

    ingeek_ecp_point_init( &R );
    ingeek_mpi_init( &k ); ingeek_mpi_init( &e ); ingeek_mpi_init( &t );

    sign_tries = 0;
    do
    {
        /*
         * Steps 1-3: generate a suitable ephemeral keypair
         * and set r = xR mod n
         */
        key_tries = 0;
        do
        {
            INGEEK_MPI_CHK( ingeek_ecp_gen_keypair( grp, &k, &R, f_rng, p_rng ) );
            INGEEK_MPI_CHK( ingeek_mpi_mod_mpi( r, &R.X, &grp->N ) );

            if( key_tries++ > 10 )
            {
                ret = INGEEK_ERR_ECP_RANDOM_FAILED;
                goto cleanup;
            }
        }
        while( ingeek_mpi_cmp_int( r, 0 ) == 0 );

        /*
         * Step 5: derive MPI from hashed message
         */
        INGEEK_MPI_CHK( derive_mpi( grp, &e, buf, blen ) );

        /*
         * Generate a random value to blind inv_mod in next step,
         * avoiding a potential timing leak.
         */
        blind_tries = 0;
        do
        {
            size_t n_size = ( grp->nbits + 7 ) / 8;
            INGEEK_MPI_CHK( ingeek_mpi_fill_random( &t, n_size, f_rng, p_rng ) );
            INGEEK_MPI_CHK( ingeek_mpi_shift_r( &t, 8 * n_size - grp->nbits ) );

            /* See ingeek_ecp_gen_keypair() */
            if( ++blind_tries > 30 )
                return( INGEEK_ERR_ECP_RANDOM_FAILED );
        }
        while( ingeek_mpi_cmp_int( &t, 1 ) < 0 ||
               ingeek_mpi_cmp_mpi( &t, &grp->N ) >= 0 );

        /*
         * Step 6: compute s = (e + r * d) / k = t (e + rd) / (kt) mod n
         */
        INGEEK_MPI_CHK( ingeek_mpi_mul_mpi( s, r, d ) );
        INGEEK_MPI_CHK( ingeek_mpi_add_mpi( &e, &e, s ) );
        INGEEK_MPI_CHK( ingeek_mpi_mul_mpi( &e, &e, &t ) );
        INGEEK_MPI_CHK( ingeek_mpi_mul_mpi( &k, &k, &t ) );
        INGEEK_MPI_CHK( ingeek_mpi_inv_mod( s, &k, &grp->N ) );
        INGEEK_MPI_CHK( ingeek_mpi_mul_mpi( s, s, &e ) );
        INGEEK_MPI_CHK( ingeek_mpi_mod_mpi( s, s, &grp->N ) );

        if( sign_tries++ > 10 )
        {
            ret = INGEEK_ERR_ECP_RANDOM_FAILED;
            goto cleanup;
        }
    }
    while( ingeek_mpi_cmp_int( s, 0 ) == 0 );

cleanup:
    ingeek_ecp_point_free( &R );
    ingeek_mpi_free( &k ); ingeek_mpi_free( &e ); ingeek_mpi_free( &t );

    return( ret );
}


/*
 * Verify ECDSA signature of hashed message (SEC1 4.1.4)
 * Obviously, compared to SEC1 4.1.3, we skip step 2 (hash message)
 */
int ingeek_ecdsa_verify( ingeek_ecp_group *grp,
                  const unsigned char *buf, size_t blen,
                  const ingeek_ecp_point *Q, const ingeek_mpi *r, const ingeek_mpi *s)
{
    int ret;
    ingeek_mpi e, s_inv, u1, u2;
    ingeek_ecp_point R;

    ingeek_ecp_point_init( &R );
    ingeek_mpi_init( &e ); ingeek_mpi_init( &s_inv ); ingeek_mpi_init( &u1 ); ingeek_mpi_init( &u2 );

    /* Fail cleanly on curves such as Curve25519 that can't be used for ECDSA */
    if( grp->N.p == NULL )
        return( INGEEK_ERR_ECP_BAD_INPUT_DATA );

    /*
     * Step 1: make sure r and s are in range 1..n-1
     */
    if( ingeek_mpi_cmp_int( r, 1 ) < 0 || ingeek_mpi_cmp_mpi( r, &grp->N ) >= 0 ||
        ingeek_mpi_cmp_int( s, 1 ) < 0 || ingeek_mpi_cmp_mpi( s, &grp->N ) >= 0 )
    {
        ret = INGEEK_ERR_ECP_VERIFY_FAILED;
        goto cleanup;
    }

    /*
     * Additional precaution: make sure Q is valid
     */
    INGEEK_MPI_CHK( ingeek_ecp_check_pubkey( grp, Q ) );

    /*
     * Step 3: derive MPI from hashed message
     */
    INGEEK_MPI_CHK( derive_mpi( grp, &e, buf, blen ) );

    /*
     * Step 4: u1 = e / s mod n, u2 = r / s mod n
     */
    INGEEK_MPI_CHK( ingeek_mpi_inv_mod( &s_inv, s, &grp->N ) );

    INGEEK_MPI_CHK( ingeek_mpi_mul_mpi( &u1, &e, &s_inv ) );
    INGEEK_MPI_CHK( ingeek_mpi_mod_mpi( &u1, &u1, &grp->N ) );

    INGEEK_MPI_CHK( ingeek_mpi_mul_mpi( &u2, r, &s_inv ) );
    INGEEK_MPI_CHK( ingeek_mpi_mod_mpi( &u2, &u2, &grp->N ) );

    /*
     * Step 5: R = u1 G + u2 Q
     *
     * Since we're not using any secret data, no need to pass a RNG to
     * ingeek_ecp_mul() for countermesures.
     */
    INGEEK_MPI_CHK( ingeek_ecp_muladd( grp, &R, &u1, &grp->G, &u2, Q ) );

    if( ingeek_ecp_is_zero( &R ) )
    {
        ret = INGEEK_ERR_ECP_VERIFY_FAILED;
        goto cleanup;
    }

    /*
     * Step 6: convert xR to an integer (no-op)
     * Step 7: reduce xR mod n (gives v)
     */
    INGEEK_MPI_CHK( ingeek_mpi_mod_mpi( &R.X, &R.X, &grp->N ) );

    /*
     * Step 8: check if v (that is, R.X) is equal to r
     */
    if( ingeek_mpi_cmp_mpi( &R.X, r ) != 0 )
    {
        ret = INGEEK_ERR_ECP_VERIFY_FAILED;
        goto cleanup;
    }

cleanup:
    ingeek_ecp_point_free( &R );
    ingeek_mpi_free( &e ); ingeek_mpi_free( &s_inv ); ingeek_mpi_free( &u1 ); ingeek_mpi_free( &u2 );

    return( ret );
}

/*
 * Convert a signature (given by context) to ASN.1
 */
static int ecdsa_signature_to_asn1( const ingeek_mpi *r, const ingeek_mpi *s,
                                    unsigned char *sig, size_t *slen )
{
    int ret;
    unsigned char buf[INGEEK_ECDSA_MAX_LEN];
    unsigned char *p = buf + sizeof( buf );
    size_t len = 0;

    INGEEK_ASN1_CHK_ADD( len, ingeek_asn1_write_mpi( &p, buf, s ) );
    INGEEK_ASN1_CHK_ADD( len, ingeek_asn1_write_mpi( &p, buf, r ) );

    INGEEK_ASN1_CHK_ADD( len, ingeek_asn1_write_len( &p, buf, len ) );
    INGEEK_ASN1_CHK_ADD( len, ingeek_asn1_write_tag( &p, buf,
                                       INGEEK_ASN1_CONSTRUCTED | INGEEK_ASN1_SEQUENCE ) );

    memcpy( sig, p, len );
    *slen = len;

    return( 0 );
}

/*
 * Compute and write signature
 */
int ingeek_ecdsa_write_signature( ingeek_ecdsa_context *ctx, 
                           const unsigned char *hash, size_t hlen,
                           unsigned char *sig, size_t *slen,
                           int (*f_rng)(void *, unsigned char *, size_t),
                           void *p_rng )
{
    int ret;
    ingeek_mpi r, s;

    ingeek_mpi_init( &r );
    ingeek_mpi_init( &s );

    INGEEK_MPI_CHK( ingeek_ecdsa_sign( &ctx->grp, &r, &s, &ctx->d,
                         hash, hlen, f_rng, p_rng ) );

    INGEEK_MPI_CHK( ecdsa_signature_to_asn1( &r, &s, sig, slen ) );

cleanup:
    ingeek_mpi_free( &r );
    ingeek_mpi_free( &s );

    return( ret );
}


/*
 * Read and check signature
 */
int ingeek_ecdsa_read_signature( ingeek_ecdsa_context *ctx,
                          const unsigned char *hash, size_t hlen,
                          const unsigned char *sig, size_t slen )
{
    int ret;
    unsigned char *p = (unsigned char *) sig;
    const unsigned char *end = sig + slen;
    size_t len;
    ingeek_mpi r, s;

    ingeek_mpi_init( &r );
    ingeek_mpi_init( &s );

    if( ( ret = ingeek_asn1_get_tag( &p, end, &len,
                    INGEEK_ASN1_CONSTRUCTED | INGEEK_ASN1_SEQUENCE ) ) != 0 )
    {
        ret += INGEEK_ERR_ECP_BAD_INPUT_DATA;
        goto cleanup;
    }

    if( p + len != end )
    {
        ret = INGEEK_ERR_ECP_BAD_INPUT_DATA +
              INGEEK_ERR_ASN1_LENGTH_MISMATCH;
        goto cleanup;
    }

    if( ( ret = ingeek_asn1_get_mpi( &p, end, &r ) ) != 0 ||
        ( ret = ingeek_asn1_get_mpi( &p, end, &s ) ) != 0 )
    {
        ret += INGEEK_ERR_ECP_BAD_INPUT_DATA;
        goto cleanup;
    }

    if( ( ret = ingeek_ecdsa_verify( &ctx->grp, hash, hlen,
                              &ctx->Q, &r, &s ) ) != 0 )
        goto cleanup;

    if( p != end )
        ret = INGEEK_ERR_ECP_SIG_LEN_MISMATCH;

cleanup:
    ingeek_mpi_free( &r );
    ingeek_mpi_free( &s );

    return( ret );
}

/*
 * Generate key pair
 */
int ingeek_ecdsa_genkey( ingeek_ecdsa_context *ctx, ingeek_ecp_group_id gid,
                  int (*f_rng)(void *, unsigned char *, size_t), void *p_rng )
{
    return( ingeek_ecp_group_load( &ctx->grp, gid ) ||
            ingeek_ecp_gen_keypair( &ctx->grp, &ctx->d, &ctx->Q, f_rng, p_rng ) );
}

/*
 * Set context from an ingeek_ecp_keypair
 */
int ingeek_ecdsa_from_keypair( ingeek_ecdsa_context *ctx, const ingeek_ecp_keypair *key )
{
    int ret;

    if( ( ret = ingeek_ecp_group_copy( &ctx->grp, &key->grp ) ) != 0 ||
        ( ret = ingeek_mpi_copy( &ctx->d, &key->d ) ) != 0 ||
        ( ret = ingeek_ecp_copy( &ctx->Q, &key->Q ) ) != 0 )
    {
        ingeek_ecdsa_free( ctx );
    }

    return( ret );
}

/*
 * Initialize context
 */
void ingeek_ecdsa_init( ingeek_ecdsa_context *ctx )
{
    ingeek_ecp_keypair_init( ctx );
}

/*
 * Free context
 */
void ingeek_ecdsa_free( ingeek_ecdsa_context *ctx )
{
    ingeek_ecp_keypair_free( ctx );
}

#endif /*#if !defined(INGEEK_SMALL_CIPHER_C)*/
#endif /* INGEEK_ECDSA_C */
