#if !defined(INGEEK_CONFIG_FILE)
#include "config.h"
#else
#include INGEEK_CONFIG_FILE
#endif

#include <stdlib.h>
#include <string.h>
#include "cipher_export.h"
#if !defined(INGEEK_SMALL_CIPHER_C)

#include "cipher_wrap.h"


#if defined(INGEEK_CMAC_C)
#include "cmac.h"
#endif

#if defined(INGEEK_PLATFORM_C)
#include "platform.h"
#else
#define ingeek_calloc calloc
#define ingeek_free   free
#endif


/* Implementation that should never be optimized out by the compiler */
static void ingeek_zeroize( void *v, size_t n ) {
    volatile unsigned char *p = (unsigned char*)v; while( n-- ) *p++ = 0;
}


/**
 * \brief               Returns the block size of the given cipher.
 *
 * \param ctx           cipher's context. Must have been initialised.
 *
 * \return              size of the cipher's blocks, or 0 if ctx has not been
 *                      initialised.
 */
static  unsigned int ingeek_cipher_get_block_size( const ingeek_cipher_context_t *ctx )
{
    if( NULL == ctx || NULL == ctx->cipher_info )
        return 0;

    return ctx->cipher_info->block_size;
}


/**
 * \brief               Returns the size of the cipher's IV/NONCE in bytes.
 *
 * \param ctx           cipher's context. Must have been initialised.
 *
 * \return              If IV has not been set yet: (recommended) IV size
 *                      (0 for ciphers not using IV/NONCE).
 *                      If IV has already been set: actual size.
 */
static  int ingeek_cipher_get_iv_size( const ingeek_cipher_context_t *ctx )
{
    if( NULL == ctx || NULL == ctx->cipher_info )
        return 0;

    if( ctx->iv_size != 0 )
        return (int) ctx->iv_size;

    return (int) ctx->cipher_info->iv_size;
}


const ingeek_cipher_info_t *Get_cipher_info_from_type( const ingeek_cipher_type_t cipher_type )
{
    const ingeek_cipher_definition_t *def;

    for( def = ingeek_cipher_definitions; def->info != NULL; def++ )
        if( def->type == cipher_type )
            return( def->info );

    return( NULL );
}

void ingeek_cipher_init( ingeek_cipher_context_t *ctx )
{
    memset( ctx, 0, sizeof( ingeek_cipher_context_t ) );
}

void ingeek_cipher_free( ingeek_cipher_context_t *ctx )
{
    if( ctx == NULL )
        return;

#if defined(INGEEK_CMAC_C)
    if( ctx->cmac_ctx )
    {
       ingeek_zeroize( ctx->cmac_ctx, sizeof( ingeek_cmac_context_t ) );
       ingeek_free( ctx->cmac_ctx );
    }
#endif

    if( ctx->cipher_ctx )
        ctx->cipher_info->base->ctx_free_func( ctx->cipher_ctx );

    ingeek_zeroize( ctx, sizeof(ingeek_cipher_context_t) );
}

int ingeek_cipher_setup( ingeek_cipher_context_t *ctx, const ingeek_cipher_info_t *cipher_info )
{
    if( NULL == cipher_info || NULL == ctx )
        return( INGEEK_ERR_CIPHER_BAD_INPUT_DATA );

    memset( ctx, 0, sizeof( ingeek_cipher_context_t ) );

    if( NULL == ( ctx->cipher_ctx = cipher_info->base->ctx_alloc_func() ) )
        return( INGEEK_ERR_CIPHER_ALLOC_FAILED );

    ctx->cipher_info = cipher_info;

#if defined(INGEEK_CIPHER_MODE_WITH_PADDING)
    /*
     * Ignore possible errors caused by a cipher mode that doesn't use padding
     */
#if defined(INGEEK_CIPHER_PADDING_PKCS7)
    (void) ingeek_cipher_set_padding_mode( ctx, INGEEK_PADDING_PKCS7 );
#else
    (void) ingeek_cipher_set_padding_mode( ctx, INGEEK_PADDING_NONE );
#endif
#endif /* INGEEK_CIPHER_MODE_WITH_PADDING */

    return( 0 );
}

int ingeek_cipher_setkey( ingeek_cipher_context_t *ctx, const unsigned char *key,
        int key_bitlen, const ingeek_operation_t operation )
{
    if( NULL == ctx || NULL == ctx->cipher_info )
        return( INGEEK_ERR_CIPHER_BAD_INPUT_DATA );

    if( ( ctx->cipher_info->flags & INGEEK_CIPHER_VARIABLE_KEY_LEN ) == 0 &&
        (int) ctx->cipher_info->key_bitlen != key_bitlen )
    {
        return( INGEEK_ERR_CIPHER_BAD_INPUT_DATA );
    }

    ctx->key_bitlen = key_bitlen;
    ctx->operation = operation;

    /*
     * For CFB and CTR mode always use the encryption key schedule
     */
    if( INGEEK_ENCRYPT == operation)
    {
        return ctx->cipher_info->base->setkey_enc_func( ctx->cipher_ctx, key,
                ctx->key_bitlen );
    }

    if( INGEEK_DECRYPT == operation )
        return ctx->cipher_info->base->setkey_dec_func( ctx->cipher_ctx, key,
                ctx->key_bitlen );

    return( INGEEK_ERR_CIPHER_BAD_INPUT_DATA );
}

int ingeek_cipher_set_iv( ingeek_cipher_context_t *ctx,
                   const unsigned char *iv, size_t iv_len )
{
    size_t actual_iv_size;

    if( NULL == ctx || NULL == ctx->cipher_info || NULL == iv )
        return( INGEEK_ERR_CIPHER_BAD_INPUT_DATA );

    /* avoid buffer overflow in ctx->iv */
    if( iv_len > INGEEK_MAX_IV_LENGTH )
        return( INGEEK_ERR_CIPHER_FEATURE_UNAVAILABLE );

    if( ( ctx->cipher_info->flags & INGEEK_CIPHER_VARIABLE_IV_LEN ) != 0 )
        actual_iv_size = iv_len;
    else
    {
        actual_iv_size = ctx->cipher_info->iv_size;

        /* avoid reading past the end of input buffer */
        if( actual_iv_size > iv_len )
            return( INGEEK_ERR_CIPHER_BAD_INPUT_DATA );
    }

    memcpy( ctx->iv, iv, actual_iv_size );
    ctx->iv_size = actual_iv_size;

    return( 0 );
}

static int ingeek_cipher_reset( ingeek_cipher_context_t *ctx )
{
    if( NULL == ctx || NULL == ctx->cipher_info )
        return( INGEEK_ERR_CIPHER_BAD_INPUT_DATA );

    ctx->unprocessed_len = 0;

    return( 0 );
}


int ingeek_cipher_update( ingeek_cipher_context_t *ctx, const unsigned char *input,
                   size_t ilen, unsigned char *output, size_t *olen )
{
    int ret;
    size_t block_size = 0;

    if( NULL == ctx || NULL == ctx->cipher_info || NULL == olen )
    {
        return( INGEEK_ERR_CIPHER_BAD_INPUT_DATA );
    }

    *olen = 0;
    block_size = ingeek_cipher_get_block_size( ctx );

    if( ctx->cipher_info->mode == INGEEK_MODE_ECB )
    {
        if( ilen != block_size )
            return( INGEEK_ERR_CIPHER_FULL_BLOCK_EXPECTED );

        *olen = ilen;

        if( 0 != ( ret = ctx->cipher_info->base->ecb_func( ctx->cipher_ctx,
                    ctx->operation, input, output ) ) )
        {
            return( ret );
        }

        return( 0 );
    }


    if ( 0 == block_size )
    {
        return INGEEK_ERR_CIPHER_INVALID_CONTEXT;
    }

    if( input == output &&
       ( ctx->unprocessed_len != 0 || ilen % block_size ) )
    {
        return( INGEEK_ERR_CIPHER_BAD_INPUT_DATA );
    }

#if defined(INGEEK_CIPHER_MODE_CBC)
    if( ctx->cipher_info->mode == INGEEK_MODE_CBC )
    {
        size_t copy_len = 0;

        /*
         * If there is not enough data for a full block, cache it.
         */
        if( ( ctx->operation == INGEEK_DECRYPT &&
                ilen <= block_size - ctx->unprocessed_len ) ||
             ( ctx->operation == INGEEK_ENCRYPT &&
                ilen < block_size - ctx->unprocessed_len ) )
        {
            memcpy( &( ctx->unprocessed_data[ctx->unprocessed_len] ), input,
                    ilen );

            ctx->unprocessed_len += ilen;
            return( 0 );
        }

        /*
         * Process cached data first
         */
        if( 0 != ctx->unprocessed_len )
        {
            copy_len = block_size - ctx->unprocessed_len;

            memcpy( &( ctx->unprocessed_data[ctx->unprocessed_len] ), input,
                    copy_len );

            if( 0 != ( ret = ctx->cipher_info->base->cbc_func( ctx->cipher_ctx,
                    ctx->operation, block_size, ctx->iv,
                    ctx->unprocessed_data, output ) ) )
            {
                return( ret );
            }

            *olen += block_size;
            output += block_size;
            ctx->unprocessed_len = 0;

            input += copy_len;
            ilen -= copy_len;
        }

        /*
         * Cache final, incomplete block
         */
        if( 0 != ilen )
        {
            if( 0 == block_size )
            {
                return INGEEK_ERR_CIPHER_INVALID_CONTEXT;
            }

            copy_len = ilen % block_size;
            if( copy_len == 0 && ctx->operation == INGEEK_DECRYPT )
                copy_len = block_size;

            memcpy( ctx->unprocessed_data, &( input[ilen - copy_len] ),
                    copy_len );

            ctx->unprocessed_len += copy_len;
            ilen -= copy_len;
        }

        /*
         * Process remaining full blocks
         */
        if( ilen )
        {
            if( 0 != ( ret = ctx->cipher_info->base->cbc_func( ctx->cipher_ctx,
                    ctx->operation, ilen, ctx->iv, input, output ) ) )
            {
                return( ret );
            }

            *olen += ilen;
        }

        return( 0 );
    }
#endif /* INGEEK_CIPHER_MODE_CBC */

    return( INGEEK_ERR_CIPHER_FEATURE_UNAVAILABLE );
}

#if defined(INGEEK_CIPHER_MODE_WITH_PADDING)
#if defined(INGEEK_CIPHER_PADDING_PKCS7)
/*
 * PKCS7 (and PKCS5) padding: fill with ll bytes, with ll = padding_len
 */
static void add_pkcs_padding( unsigned char *output, size_t output_len,
        size_t data_len )
{
    size_t padding_len = output_len - data_len;
    unsigned char i;

    for( i = 0; i < padding_len; i++ )
        output[data_len + i] = (unsigned char) padding_len;
}

static int get_pkcs_padding( unsigned char *input, size_t input_len,
        size_t *data_len )
{
    size_t i, pad_idx;
    unsigned char padding_len, bad = 0;

    if( NULL == input || NULL == data_len )
        return( INGEEK_ERR_CIPHER_BAD_INPUT_DATA );

    padding_len = input[input_len - 1];
    *data_len = input_len - padding_len;

    /* Avoid logical || since it results in a branch */
    bad |= padding_len > input_len;
    bad |= padding_len == 0;

    /* The number of bytes checked must be independent of padding_len,
     * so pick input_len, which is usually 8 or 16 (one block) */
    pad_idx = input_len - padding_len;
    for( i = 0; i < input_len; i++ )
        bad |= ( input[i] ^ padding_len ) * ( i >= pad_idx );

    return( INGEEK_ERR_CIPHER_INVALID_PADDING * ( bad != 0 ) );
}
#endif /* INGEEK_CIPHER_PADDING_PKCS7 */

#if defined(INGEEK_CIPHER_PADDING_ONE_AND_ZEROS)
/*
 * One and zeros padding: fill with 80 00 ... 00
 */
static void add_one_and_zeros_padding( unsigned char *output,
                                       size_t output_len, size_t data_len )
{
    size_t padding_len = output_len - data_len;
    unsigned char i = 0;

    output[data_len] = 0x80;
    for( i = 1; i < padding_len; i++ )
        output[data_len + i] = 0x00;
}

static int get_one_and_zeros_padding( unsigned char *input, size_t input_len,
                                      size_t *data_len )
{
    size_t i;
    unsigned char done = 0, prev_done, bad;

    if( NULL == input || NULL == data_len )
        return( INGEEK_ERR_CIPHER_BAD_INPUT_DATA );

    bad = 0xFF;
    *data_len = 0;
    for( i = input_len; i > 0; i-- )
    {
        prev_done = done;
        done |= ( input[i-1] != 0 );
        *data_len |= ( i - 1 ) * ( done != prev_done );
        bad &= ( input[i-1] ^ 0x80 ) | ( done == prev_done );
    }

    return( INGEEK_ERR_CIPHER_INVALID_PADDING * ( bad != 0 ) );

}
#endif /* INGEEK_CIPHER_PADDING_ONE_AND_ZEROS */

#if defined(INGEEK_CIPHER_PADDING_ZEROS_AND_LEN)
/*
 * Zeros and len padding: fill with 00 ... 00 ll, where ll is padding length
 */
static void add_zeros_and_len_padding( unsigned char *output,
                                       size_t output_len, size_t data_len )
{
    size_t padding_len = output_len - data_len;
    unsigned char i = 0;

    for( i = 1; i < padding_len; i++ )
        output[data_len + i - 1] = 0x00;
    output[output_len - 1] = (unsigned char) padding_len;
}

static int get_zeros_and_len_padding( unsigned char *input, size_t input_len,
                                      size_t *data_len )
{
    size_t i, pad_idx;
    unsigned char padding_len, bad = 0;

    if( NULL == input || NULL == data_len )
        return( INGEEK_ERR_CIPHER_BAD_INPUT_DATA );

    padding_len = input[input_len - 1];
    *data_len = input_len - padding_len;

    /* Avoid logical || since it results in a branch */
    bad |= padding_len > input_len;
    bad |= padding_len == 0;

    /* The number of bytes checked must be independent of padding_len */
    pad_idx = input_len - padding_len;
    for( i = 0; i < input_len - 1; i++ )
        bad |= input[i] * ( i >= pad_idx );

    return( INGEEK_ERR_CIPHER_INVALID_PADDING * ( bad != 0 ) );
}
#endif /* INGEEK_CIPHER_PADDING_ZEROS_AND_LEN */

#if defined(INGEEK_CIPHER_PADDING_ZEROS)
/*
 * Zero padding: fill with 00 ... 00
 */
static void add_zeros_padding( unsigned char *output,
                               size_t output_len, size_t data_len )
{
    size_t i;

    for( i = data_len; i < output_len; i++ )
        output[i] = 0x00;
}

static int get_zeros_padding( unsigned char *input, size_t input_len,
                              size_t *data_len )
{
    size_t i;
    unsigned char done = 0, prev_done;

    if( NULL == input || NULL == data_len )
        return( INGEEK_ERR_CIPHER_BAD_INPUT_DATA );

    *data_len = 0;
    for( i = input_len; i > 0; i-- )
    {
        prev_done = done;
        done |= ( input[i-1] != 0 );
        *data_len |= i * ( done != prev_done );
    }

    return( 0 );
}
#endif /* INGEEK_CIPHER_PADDING_ZEROS */

/*
 * No padding: don't pad :)
 *
 * There is no add_padding function (check for NULL in ingeek_cipher_finish)
 * but a trivial get_padding function
 */
static int get_no_padding( unsigned char *input, size_t input_len,
                              size_t *data_len )
{
    if( NULL == input || NULL == data_len )
        return( INGEEK_ERR_CIPHER_BAD_INPUT_DATA );

    *data_len = input_len;

    return( 0 );
}
#endif /* INGEEK_CIPHER_MODE_WITH_PADDING */

int ingeek_cipher_finish( ingeek_cipher_context_t *ctx,
                   unsigned char *output, size_t *olen )
{
    if( NULL == ctx || NULL == ctx->cipher_info || NULL == olen )
        return( INGEEK_ERR_CIPHER_BAD_INPUT_DATA );

    *olen = 0;

    if( INGEEK_MODE_ECB == ctx->cipher_info->mode )
    {
        if( ctx->unprocessed_len != 0 )
            return( INGEEK_ERR_CIPHER_FULL_BLOCK_EXPECTED );

        return( 0 );
    }

#if defined(INGEEK_CIPHER_MODE_CBC)
    if( INGEEK_MODE_CBC == ctx->cipher_info->mode )
    {
        int ret = 0;

        if( INGEEK_ENCRYPT == ctx->operation )
        {
            /* check for 'no padding' mode */
            if( NULL == ctx->add_padding )
            {
                if( 0 != ctx->unprocessed_len )
                    return( INGEEK_ERR_CIPHER_FULL_BLOCK_EXPECTED );

                return( 0 );
            }

            ctx->add_padding( ctx->unprocessed_data, ingeek_cipher_get_iv_size( ctx ),
                    ctx->unprocessed_len );
        }
        else if( ingeek_cipher_get_block_size( ctx ) != ctx->unprocessed_len )
        {
            /*
             * For decrypt operations, expect a full block,
             * or an empty block if no padding
             */
            if( NULL == ctx->add_padding && 0 == ctx->unprocessed_len )
                return( 0 );

            return( INGEEK_ERR_CIPHER_FULL_BLOCK_EXPECTED );
        }

        /* cipher block */
        if( 0 != ( ret = ctx->cipher_info->base->cbc_func( ctx->cipher_ctx,
                ctx->operation, ingeek_cipher_get_block_size( ctx ), ctx->iv,
                ctx->unprocessed_data, output ) ) )
        {
            return( ret );
        }

        /* Set output size for decryption */
        if( INGEEK_DECRYPT == ctx->operation )
            return ctx->get_padding( output, ingeek_cipher_get_block_size( ctx ),
                                     olen );

        /* Set output size for encryption */
        *olen = ingeek_cipher_get_block_size( ctx );
        return( 0 );
    }
#else
    ((void) output);
#endif /* INGEEK_CIPHER_MODE_CBC */

    return( INGEEK_ERR_CIPHER_FEATURE_UNAVAILABLE );
}

#if defined(INGEEK_CIPHER_MODE_WITH_PADDING)
int ingeek_cipher_set_padding_mode( ingeek_cipher_context_t *ctx, ingeek_cipher_padding_t mode )
{
    if( NULL == ctx ||
        INGEEK_MODE_CBC != ctx->cipher_info->mode )
    {
        return( INGEEK_ERR_CIPHER_BAD_INPUT_DATA );
    }

    switch( mode )
    {
#if defined(INGEEK_CIPHER_PADDING_PKCS7)
    case INGEEK_PADDING_PKCS7:
        ctx->add_padding = add_pkcs_padding;
        ctx->get_padding = get_pkcs_padding;
        break;
#endif
#if defined(INGEEK_CIPHER_PADDING_ONE_AND_ZEROS)
    case INGEEK_PADDING_ONE_AND_ZEROS:
        ctx->add_padding = add_one_and_zeros_padding;
        ctx->get_padding = get_one_and_zeros_padding;
        break;
#endif
#if defined(INGEEK_CIPHER_PADDING_ZEROS_AND_LEN)
    case INGEEK_PADDING_ZEROS_AND_LEN:
        ctx->add_padding = add_zeros_and_len_padding;
        ctx->get_padding = get_zeros_and_len_padding;
        break;
#endif
#if defined(INGEEK_CIPHER_PADDING_ZEROS)
    case INGEEK_PADDING_ZEROS:
        ctx->add_padding = add_zeros_padding;
        ctx->get_padding = get_zeros_padding;
        break;
#endif
    case INGEEK_PADDING_NONE:
        ctx->add_padding = NULL;
        ctx->get_padding = get_no_padding;
        break;

    default:
        return( INGEEK_ERR_CIPHER_FEATURE_UNAVAILABLE );
    }

    return( 0 );
}
#endif /* INGEEK_CIPHER_MODE_WITH_PADDING */


/*
 * Packet-oriented wrapper for non-AEAD modes
 */
int ingeek_cipher_crypt( ingeek_cipher_context_t *ctx,
                  const unsigned char *iv, size_t iv_len,
                  const unsigned char *input, size_t ilen,
                  unsigned char *output, size_t *olen )
{
    int ret;
    size_t finish_olen;

    if( ( ret = ingeek_cipher_set_iv( ctx, iv, iv_len ) ) != 0 )
        return( ret );

    if( ( ret = ingeek_cipher_reset( ctx ) ) != 0 )
        return( ret );

    if( ( ret = ingeek_cipher_update( ctx, input, ilen, output, olen ) ) != 0 )
        return( ret );

    if( ( ret = ingeek_cipher_finish( ctx, output + *olen, &finish_olen ) ) != 0 )
        return( ret );

    *olen += finish_olen;

    return( 0 );
}

#endif  /*INGEEK_SMALL_CIPHER_C*/

#include "aes.h"

int ingeek_aes_ecb_encrypt(unsigned char *key,unsigned char*input,size_t inlen,unsigned char*output)
{
	int ret=0;
	
#if !defined(INGEEK_SMALL_CIPHER_C)

	size_t outlen=0;
	size_t finish_olen;
    int keylen = 0;
    const ingeek_cipher_info_t *cipher_info;
    ingeek_cipher_context_t cipher_ctx;
	
	cipher_info = Get_cipher_info_from_type(INGEEK_CIPHER_AES_128_ECB);
    if( cipher_info == NULL )
        return  -1;
	
	keylen = cipher_info->key_bitlen / 8;
	ingeek_cipher_init( &cipher_ctx );
	
	if((ret = ingeek_cipher_setup( &cipher_ctx, cipher_info)) != 0)
		return (ret);
	
	if((ret = ingeek_cipher_setkey( &cipher_ctx, key, 8*keylen, INGEEK_ENCRYPT)) != 0)
		return (ret);
	
    if( ( ret = ingeek_cipher_update(&cipher_ctx, input, inlen, output, &outlen ) ) != 0 )
        return( ret );

    if( ( ret = ingeek_cipher_finish(&cipher_ctx, output + outlen, &finish_olen ) ) != 0 )
        return( ret );

    outlen += finish_olen;
	
	ingeek_cipher_free( &cipher_ctx );
#else
	AES_ECB_encrypt(input, key, output, inlen);
	
#endif
	return ret;
}

#if defined(INGEEK_SMALL_CIPHER_C)
#if defined(CIPHER_PADDING_PKCS7)
static int PKCS7_PadData(unsigned char* in, int inSz, unsigned char* out, int outSz,int blockSz)
{
    int i, padSz;

    if (in == NULL  || inSz == 0 ||
        out == NULL || outSz == 0)
        return -1;

    padSz = blockSz - (inSz % blockSz);

    if (outSz < (inSz + padSz))
        return -1;

    memcpy(out, in, inSz);

    for (i = 0; i < padSz; i++) {
        out[inSz + i] = (char)padSz;
    }

    return inSz + padSz;
}


static int PKCS7_UnPad(unsigned char *data, int datalen)
{
    int	 padlen;
    unsigned char*pos;

    padlen = (unsigned char)data[datalen - 1];
    pos = data + datalen - padlen;
    *pos = '\0';

    return padlen;
}
#endif /*CIPHER_PADDING_PKCS7*/
#endif /*INGEEK_SMALL_CIPHER_C*/


int ingeek_cipher_aes_cbc(unsigned char* key,unsigned char*iv,
							unsigned char*in,unsigned int ilen,
							unsigned char*out,unsigned int *olen,
							ingeek_operation_t mode)
{
    int ret = 0;
	#if defined(INGEEK_SMALL_CIPHER_C)
    unsigned int encrypte_len = 0;
    unsigned char psout[CIPHER_LEN]={0};
	#endif
	if (key == NULL || iv == NULL || in == NULL || ilen == 0){
              return -1;
    }

#if !defined(INGEEK_SMALL_CIPHER_C)
    size_t keylen=0;
    const ingeek_cipher_info_t *cipher_info;
	ingeek_cipher_context_t cipher_ctx;
	
	
	cipher_info = Get_cipher_info_from_type(INGEEK_CIPHER_AES_128_CBC);
    if( cipher_info == NULL )
        return  -1;
	
	keylen = cipher_info->key_bitlen / 8;
	ingeek_cipher_init( &cipher_ctx );
	
	ret = ingeek_cipher_setup( &cipher_ctx, cipher_info);
	if(ret != 0){
		ret= -1;
	}
	ret = ingeek_cipher_setkey( &cipher_ctx, key, 8 * keylen, (ingeek_operation_t) mode);
	if(ret != 0){
		ret=-2;
	}
	ret = ingeek_cipher_crypt( &cipher_ctx, iv, 16,in, ilen, out, olen );
	if(ret != 0){
		ret=-3;
	}
	
	ingeek_cipher_free( &cipher_ctx );
#else
	  
      if(mode != INGEEK_DECRYPT){
#if defined(CIPHER_PADDING_PKCS7)
              encrypte_len = (ilen + BLOCKLEN)&(~(BLOCKLEN - 1));
              PKCS7_PadData(in, ilen,psout,encrypte_len,BLOCKLEN);
			  AES_CBC_encrypt_buffer(out, psout, encrypte_len, key, iv);
              *olen=encrypte_len;
#endif
              ret=0;
      }
      else{
#if defined(CIPHER_PADDING_PKCS7)
			  AES_CBC_decrypt_buffer(out, in, ilen, key, iv);
              encrypte_len=PKCS7_UnPad(out,ilen);
              *olen=ilen-encrypte_len;
#endif
			ret=0;
      }
	  
#endif
	return ret;
	
}


