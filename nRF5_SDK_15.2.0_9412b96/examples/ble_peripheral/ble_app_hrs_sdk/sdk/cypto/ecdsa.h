#ifndef INGEEK_ECDSA_H
#define INGEEK_ECDSA_H
#if !defined(INGEEK_CONFIG_FILE)
#include "config.h"
#else
#include INGEEK_CONFIG_FILE
#endif

#if !defined(INGEEK_SMALL_CIPHER_C)

#include "ecp.h"

#if INGEEK_ECP_MAX_BYTES > 124
#error "INGEEK_ECP_MAX_BYTES bigger than expected, please fix INGEEK_ECDSA_MAX_LEN"
#endif

/** Maximum size of an ECDSA signature in bytes */
#define INGEEK_ECDSA_MAX_LEN  ( 3 + 2 * ( 3 + INGEEK_ECP_MAX_BYTES ) )

typedef ingeek_ecp_keypair ingeek_ecdsa_context;

#ifdef __cplusplus
extern "C" {
#endif


/**
 * \brief           Compute ECDSA signature and write it to buffer,
 *                  serialized as defined in RFC 4492 page 20.
 *                  (Not thread-safe to use same context in multiple threads)
 *
 * \note            The deterministic version (RFC 6979) is used if
 *                  INGEEK_ECDSA_DETERMINISTIC is defined.
 *
 * \param ctx       ECDSA context
 * \param md_alg    Algorithm that was used to hash the message
 * \param hash      Message hash
 * \param hlen      Length of hash
 * \param sig       Buffer that will hold the signature
 * \param slen      Length of the signature written
 * \param f_rng     RNG function
 * \param p_rng     RNG parameter
 *
 * \note            The "sig" buffer must be at least as large as twice the
 *                  size of the curve used, plus 9 (eg. 73 bytes if a 256-bit
 *                  curve is used). INGEEK_ECDSA_MAX_LEN is always safe.
 *
 * \note            If the bitlength of the message hash is larger than the
 *                  bitlength of the group order, then the hash is truncated as
 *                  prescribed by SEC1 4.1.3 step 5.
 *
 * \return          0 if successful,
 *                  or a INGEEK_ERR_ECP_XXX, INGEEK_ERR_MPI_XXX or
 *                  INGEEK_ERR_ASN1_XXX error code
 */
int ingeek_ecdsa_write_signature( ingeek_ecdsa_context *ctx,
                           const unsigned char *hash, size_t hlen,
                           unsigned char *sig, size_t *slen,
                           int (*f_rng)(void *, unsigned char *, size_t),
                           void *p_rng );

/**
 * \brief           Read and verify an ECDSA signature
 *
 * \param ctx       ECDSA context
 * \param hash      Message hash
 * \param hlen      Size of hash
 * \param sig       Signature to read and verify
 * \param slen      Size of sig
 *
 * \note            If the bitlength of the message hash is larger than the
 *                  bitlength of the group order, then the hash is truncated as
 *                  prescribed by SEC1 4.1.4 step 3.
 *
 * \return          0 if successful,
 *                  INGEEK_ERR_ECP_BAD_INPUT_DATA if signature is invalid,
 *                  INGEEK_ERR_ECP_SIG_LEN_MISMATCH if the signature is
 *                  valid but its actual length is less than siglen,
 *                  or a INGEEK_ERR_ECP_XXX or INGEEK_ERR_MPI_XXX error code
 */
int ingeek_ecdsa_read_signature( ingeek_ecdsa_context *ctx,
                          const unsigned char *hash, size_t hlen,
                          const unsigned char *sig, size_t slen );

/**
 * \brief           Generate an ECDSA keypair on the given curve
 *
 * \param ctx       ECDSA context in which the keypair should be stored
 * \param gid       Group (elliptic curve) to use. One of the various
 *                  INGEEK_ECP_DP_XXX macros depending on configuration.
 * \param f_rng     RNG function
 * \param p_rng     RNG parameter
 *
 * \return          0 on success, or a INGEEK_ERR_ECP_XXX code.
 */
int ingeek_ecdsa_genkey( ingeek_ecdsa_context *ctx, ingeek_ecp_group_id gid,
                  int (*f_rng)(void *, unsigned char *, size_t), void *p_rng );

/**
 * \brief           Set an ECDSA context from an EC key pair
 *
 * \param ctx       ECDSA context to set
 * \param key       EC key to use
 *
 * \return          0 on success, or a INGEEK_ERR_ECP_XXX code.
 */
int ingeek_ecdsa_from_keypair( ingeek_ecdsa_context *ctx, const ingeek_ecp_keypair *key );

/**
 * \brief           Initialize context
 *
 * \param ctx       Context to initialize
 */
void ingeek_ecdsa_init( ingeek_ecdsa_context *ctx );

/**
 * \brief           Free context
 *
 * \param ctx       Context to free
 */
void ingeek_ecdsa_free( ingeek_ecdsa_context *ctx );

#ifdef __cplusplus
}
#endif

#endif /*#if !defined(INGEEK_SMALL_CIPHER_C)*/
#endif /* ecdsa.h */
