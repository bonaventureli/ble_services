#include <stdio.h>
#include <string.h>
#include "cipher_export.h"
#include "cmac.h"
#include "printf.h"
#include "nrf_log.h"//add lifei
typedef struct{
    unsigned char key[AES_BLOCK_SIZE];
    unsigned char buffer[AES_BLOCK_SIZE]; /* partially stored block */
    unsigned char digest[AES_BLOCK_SIZE]; /* running digest */
    unsigned char k1[AES_BLOCK_SIZE];
    unsigned char k2[AES_BLOCK_SIZE];
    int bufferSz;
    int totalSz;
} Cmac;


typedef enum {
    CMAC_AES = 1
} CmacType;



static void xorbuf(void* buf, const void* mask, int count)
{

    int i;
    unsigned char*       b = (unsigned char*)buf;
    const unsigned char* m = (const unsigned char*)mask;

    for (i = 0; i < count; i++) b[i] ^= m[i];

}


/* Make sure compiler doesn't skip */
static void ForceZero(const void* mem, int len)
{
    volatile unsigned char* z = (volatile unsigned char*)mem;
    while (len--) *z++ = 0;
}


/* check all length unsigned chars for equality, return 0 on success */
static int ConstantCompare(const unsigned char* a, const unsigned char* b, int length)
{
    int i;
    int compareSum = 0;
    for (i = 0; i < length; i++) {
        //compareSum |= a[i] ^ b[i];
	if(a[i] != b[i])
	{
	    compareSum = 1;
	    break;
	}
    }
    return compareSum;
}
   
static void ShiftAndXorRb(unsigned char* out, unsigned char* in)
{
    int i, j, xorRb;
    int mask = 0, last = 0;
    unsigned char Rb = 0x87;

    xorRb = (in[0] & 0x80) != 0;

    for (i = 1, j = AES_BLOCK_SIZE - 1; i <= AES_BLOCK_SIZE; i++, j--) {
        last = (in[j] & 0x80) ? 1 : 0;
        out[j] = (in[j] << 1) | mask;
        mask = last;
        if (xorRb) {
            out[j] ^= Rb;
            Rb = 0;
        }
    }
}



static int ingeek_InitCmac(Cmac* cmac, const unsigned char* key, int keySz,int type, void* unused)
{
    unsigned char l[AES_BLOCK_SIZE];

    if (cmac == NULL || key == NULL || keySz == 0 || type != CMAC_AES)
        return -1;

    	memset(cmac, 0, sizeof(Cmac));
	memcpy(cmac->key,key,keySz);
	
	memset(l, 0, AES_BLOCK_SIZE);
	ingeek_aes_ecb_encrypt(cmac->key,l,AES_BLOCK_SIZE,l);
	
	ShiftAndXorRb(cmac->k1, l);
	ShiftAndXorRb(cmac->k2, cmac->k1);
	ForceZero(l, AES_BLOCK_SIZE);

    return 0;
}


static int ingeek_CmacUpdate(Cmac* cmac, const unsigned char* in, int inSz)
{
    if ((cmac == NULL) || (in == NULL && inSz != 0))
        return -1;

    while (inSz != 0) {
        int add = min(inSz, AES_BLOCK_SIZE - cmac->bufferSz);
        memcpy(&cmac->buffer[cmac->bufferSz], in, add);

        cmac->bufferSz += add;
        in += add;
        inSz -= add;

        if (cmac->bufferSz == AES_BLOCK_SIZE && inSz != 0) {
            if (cmac->totalSz != 0)
                xorbuf(cmac->buffer, cmac->digest, AES_BLOCK_SIZE);
			ingeek_aes_ecb_encrypt(cmac->key,cmac->buffer,AES_BLOCK_SIZE,cmac->digest);
            cmac->totalSz += AES_BLOCK_SIZE;
            cmac->bufferSz = 0;
        }
    }

    return 0;
}


static int ingeek_CmacFinal(Cmac* cmac, unsigned char* out, int* outSz)
{
    const unsigned char* subKey;

    if (cmac == NULL || out == NULL){
        return -1;
	}
	
    if (outSz != NULL && *outSz < AES_BLOCK_SIZE){
		//printf("*outSz=%d\n",*outSz);
        return -1;
	}

    if (cmac->bufferSz == AES_BLOCK_SIZE) {
        subKey = cmac->k1;
    }
    else {
        int remainder = AES_BLOCK_SIZE - cmac->bufferSz;

        if (remainder == 0)
            remainder = AES_BLOCK_SIZE;

        if (remainder > 1)
            memset(cmac->buffer + AES_BLOCK_SIZE - remainder, 0, remainder);
        cmac->buffer[AES_BLOCK_SIZE - remainder] = 0x80;
        subKey = cmac->k2;
    }
    xorbuf(cmac->buffer, cmac->digest, AES_BLOCK_SIZE);
    xorbuf(cmac->buffer, subKey, AES_BLOCK_SIZE);

	ingeek_aes_ecb_encrypt(cmac->key,cmac->buffer,AES_BLOCK_SIZE,out);
    if (outSz != NULL)
        *outSz = AES_BLOCK_SIZE;
    ForceZero(cmac, sizeof(Cmac));

    return 0; 
}


int ingeek_AesCmacGenerate(unsigned char* out, int* outSz,
                       const unsigned char* in, int inSz,
                       const unsigned char* key, int keySz)
{
    Cmac cmac;
    int ret;

    if (out == NULL || (in == NULL && inSz > 0) || key == NULL || keySz == 0){
        return -1;
	}

    ret = ingeek_InitCmac(&cmac, key, keySz, CMAC_AES, NULL);
    if (ret != 0){
		//printf("ingeek_InitCmac\n");
        return ret;
	}

    ret = ingeek_CmacUpdate(&cmac, in, inSz);
    if (ret != 0){
		//printf("ingeek_CmacUpdate\n");
        return ret;
	}

    ret = ingeek_CmacFinal(&cmac, out, outSz);
    if (ret != 0){
		//printf("ingeek_CmacFinal\n");
        return ret;
	}

    return 0;
}


static void hex_dump(unsigned char* name, const unsigned char* buffer, int len)
{
	int uIdx=0;
	
	DebugPrintf("%s: ",name);
	for (uIdx = 0; uIdx < len; uIdx++)
	{
		DebugPrintf("%02X ", buffer[uIdx]);
	}
	DebugPrintf("\n");
}


int ingeek_AesCmacVerify(const unsigned char* check, int checkSz,
                     const unsigned char* in, int inSz,
                     const unsigned char* key, int keySz)
{
    unsigned char a[AES_BLOCK_SIZE];
    int aSz = sizeof(a);
    int result;
    int compareRet;

    if (check == NULL || checkSz == 0 || (in == NULL && inSz != 0) ||
        key == NULL || keySz == 0)
        return -1;

    memset(a, 0, aSz);
    result = ingeek_AesCmacGenerate(a, &aSz, in, inSz, key, keySz);
    compareRet = ConstantCompare(check, a, AES_BLOCK_SIZE);
    
		NRF_LOG_INFO("inSz %d",inSz);
		NRF_LOG_HEXDUMP_INFO( in, inSz); //add 
		NRF_LOG_INFO("keySz %d",keySz);
    NRF_LOG_HEXDUMP_INFO( key, keySz); //add 
		NRF_LOG_INFO("AES_BLOCK_SIZE %d",AES_BLOCK_SIZE);
    NRF_LOG_HEXDUMP_INFO( check, AES_BLOCK_SIZE); //add 
		NRF_LOG_INFO("AES_BLOCK_SIZE %d",AES_BLOCK_SIZE);
    NRF_LOG_HEXDUMP_INFO( a, AES_BLOCK_SIZE); //add 
		
		
//    hex_dump("in data is", in, inSz); //add 
//    hex_dump("key data is", key, keySz); //add 
//    hex_dump("aesverify check is", check, AES_BLOCK_SIZE); //add 
//    hex_dump("aesverify a is ", a, AES_BLOCK_SIZE); //add 

    if (result == 0){
        result = compareRet ? 1 : 0;
			
		}
		NRF_LOG_INFO("ingeek_AesCmacVerify compareRet %x",result);
    return result;
}



