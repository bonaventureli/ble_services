#include "fstorage_data.h"
#include "nrf_drv_clock.h"
//#include "nrf_fstorage.h"

//#include "fds.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#ifdef SOFTDEVICE_PRESENT

#include "nrf_sdh.h"

#include "nrf_sdh_ble.h"

#include "nrf_fstorage_sd.h"

#else

#include "nrf_drv_clock.h"

#include "nrf_fstorage_nvmc.h"

#endif




static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =

{

    /* Set a handler for fstorage events. */

    .evt_handler = fstorage_evt_handler,



    /* These below are the boundaries of the flash space assigned to this instance of fstorage.

     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.

     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the

     * last page of flash available to write data. */

    //.start_addr = 0x3e000,

    //.end_addr   = 0x3ffff,

		.start_addr = FSTORAGE_DATA_ADDR_START,

    .end_addr   = (FSTORAGE_DATA_ADDR_START + FSTORAGE_PAGE_SIZES - 1),

};

#define FSTORAGE_DATA_SIZE_IN_BYTES (256)  

//#define FSTORAGE_DATA_SIZE_IN_BYTES (192)  

//#define FSTORAGE_DATA_SIZE_IN_BYTES (128) 

//#define FSTORAGE_DATA_SIZE_IN_BYTES (64)  

__ALIGN(4) static uint8_t fstorage_data_write_temp[FSTORAGE_DATA_SIZE_IN_BYTES];





void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage);

static void clock_init(void)

{

    /* Initialize the clock. */

    ret_code_t rc = nrf_drv_clock_init();

    APP_ERROR_CHECK(rc);



    nrf_drv_clock_lfclk_request(NULL);



    // Wait for the clock to be ready.

    while (!nrf_clock_lf_is_running()) {;}

}

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)

{

    if (p_evt->result != NRF_SUCCESS)

    {

        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");

        return;

    }



    switch (p_evt->id)

    {

        case NRF_FSTORAGE_EVT_WRITE_RESULT:

        {

            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",

                         p_evt->len, p_evt->addr);

        } break;



        case NRF_FSTORAGE_EVT_ERASE_RESULT:

        {

            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",

                         p_evt->len, p_evt->addr);

        } break;



        default:

            break;

    }

}





static void print_flash_info(nrf_fstorage_t * p_fstorage)

{

    NRF_LOG_INFO("========| flash info |========");

    NRF_LOG_INFO("erase unit: \t%d bytes",      p_fstorage->p_flash_info->erase_unit);

    NRF_LOG_INFO("program unit: \t%d bytes",    p_fstorage->p_flash_info->program_unit);

    NRF_LOG_INFO("==============================");

}

static uint32_t nrf5_flash_end_addr_get()

{

    uint32_t const bootloader_addr = NRF_UICR->NRFFW[0];

    uint32_t const page_sz         = NRF_FICR->CODEPAGESIZE;

    uint32_t const code_sz         = NRF_FICR->CODESIZE;



    return (bootloader_addr != 0xFFFFFFFF ?

            bootloader_addr : (code_sz * page_sz));

}

void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)

{

    /* While fstorage is busy, sleep and wait for an event. */

    while (nrf_fstorage_is_busy(p_fstorage))

    {

        //power_manage();

    }

}
uint32_t fstorage_data_init(void){

		ret_code_t rc;

		nrf_fstorage_api_t * p_fs_api;

	   



		#ifdef SOFTDEVICE_PRESENT

    //NRF_LOG_INFO("SoftDevice is present.");

    //NRF_LOG_INFO("Initializing nrf_fstorage_sd implementation...");

    /* Initialize an fstorage instance using the nrf_fstorage_sd backend.

     * nrf_fstorage_sd uses the SoftDevice to write to flash. This implementation can safely be

     * used whenever there is a SoftDevice, regardless of its status (enabled/disabled). */

    p_fs_api = &nrf_fstorage_sd;

#else

    NRF_LOG_INFO("SoftDevice not present.");

    NRF_LOG_INFO("Initializing nrf_fstorage_nvmc implementation...");

    /* Initialize an fstorage instance using the nrf_fstorage_nvmc backend.

     * nrf_fstorage_nvmc uses the NVMC peripheral. This implementation can be used when the

     * SoftDevice is disabled or not present.

     *

     * Using this implementation when the SoftDevice is enabled results in a hardfault. */

    p_fs_api = &nrf_fstorage_nvmc;

#endif

		rc = nrf_fstorage_init(&fstorage, p_fs_api, NULL);

    APP_ERROR_CHECK(rc);

		if (NRF_SUCCESS != rc)

    {

        NRF_LOG_INFO("nrf_fstorage_init Failed, 0x%x", rc);

        return rc;

    }

		

		//print_flash_info(&fstorage);

/* It is possible to set the start and end addresses of an fstorage instance at runtime.

     * They can be set multiple times, should it be needed. The helper function below can

     * be used to determine the last address on the last page of flash memory available to

     * store data. */

		NRF_LOG_INFO("nrf5_flash_end_addr=0x%x", nrf5_flash_end_addr_get());

    //(void) nrf5_flash_end_addr_get();

		 /* Let's write to flash. */

		

		 #if 0

		//static uint32_t m_data          = 0xBADC0FFE;

		uint8_t p_src[]={0x01,0x02,0x03,0x04};

		uint8_t p_dest[4];

    //m_data = 0xDEADBEEF;



    NRF_LOG_HEXDUMP_INFO(p_src,sizeof(p_src));

    rc = nrf_fstorage_write(&fstorage, 0x3e100, p_src, sizeof(p_src), NULL);

    APP_ERROR_CHECK(rc);



    wait_for_flash_ready(&fstorage);

    NRF_LOG_INFO("Done.");

		

		rc = nrf_fstorage_read(&fstorage,0x3e100,p_dest,sizeof(p_dest));

		APP_ERROR_CHECK(rc);

		NRF_LOG_HEXDUMP_INFO(p_dest, sizeof(p_dest));	

		#endif


}
uint32_t fstorage_data_read(uint32_t offset, uint8_t * p_dest, uint32_t len)

{

    ret_code_t rc = NRF_SUCCESS;

    VERIFY_PARAM_NOT_NULL(p_dest);

    VERIFY_FALSE((len + offset > sizeof(fstorage_data_write_temp)), NRF_ERROR_INVALID_PARAM);



#if 0

    //fstorage_data_lock();



    // prepare data

    rc = nrf_fstorage_read(&fstorage, fstorage.start_addr,

                           fstorage_data_write_temp, sizeof(fstorage_data_write_temp));

    if (NRF_SUCCESS != rc)

    {

        NRF_LOG_INFO("nrf_fstorage_read fstorage init data Failed, 0x%x", rc);

        goto READ_OUT;

    }



    memcpy(p_dest, fstorage_data_write_temp + offset, len);



READ_OUT:



    //fstorage_data_unlock();

#else

		

    memcpy(p_dest,(void*) (fstorage.start_addr + offset), len);

		#if 0

		NRF_LOG_INFO("fstorage.start_addr is %x",fstorage.start_addr);

		NRF_LOG_INFO("len is %d",len);

		NRF_LOG_HEXDUMP_INFO(p_dest, len);

		#endif

#endif



    return rc;

}


#if 1
uint8_t i=0;
uint8_t fstorage_data_write_temp2[128];
uint32_t fstorage_data_write(uint32_t offset, void const * p_src, uint32_t len)

{
    ret_code_t rc = NRF_SUCCESS;
		i++;
		NRF_LOG_INFO("nrf_fstorage_erase couter: %d", i);
    //memcpy(fstorage_data_write_temp + offset,p_src,len);
	  //return rc;
    VERIFY_PARAM_NOT_NULL(p_src);
    VERIFY_FALSE((len + offset > sizeof(fstorage_data_write_temp)), NRF_ERROR_INVALID_PARAM);
    //fstorage_data_lock();
		
    // prepare data
    rc = nrf_fstorage_read(&fstorage, fstorage.start_addr+offset,fstorage_data_write_temp+offset, sizeof(fstorage_data_write_temp));
    if (NRF_SUCCESS != rc)
    {
        NRF_LOG_INFO("nrf_fstorage_read fstorage init data Failed, 0x%x", rc);
    }
    memcpy(fstorage_data_write_temp + offset, p_src, len);

		NRF_LOG_INFO("fstorage_data_write_temp + offset:",(fstorage_data_write_temp + offset));
		NRF_LOG_HEXDUMP_INFO(fstorage_data_write_temp + offset, len);
		
		NRF_LOG_INFO("fstorage_data_write_temp:",(fstorage_data_write_temp));
		NRF_LOG_HEXDUMP_INFO(fstorage_data_write_temp, len);

    // erase
    
		
		
		//if(i == 1){
		  rc = nrf_fstorage_erase(&fstorage, fstorage.start_addr, 1, NULL);
			//rc = nrf_fstorage_erase(&fstorage, fstorage.start_addr+offset, 1, NULL);
			
			if (NRF_SUCCESS != rc)
			{
					NRF_LOG_INFO("nrf_fstorage_erase Failed, rc=0x%x", rc);
			}
		//}

    //wait_for_flash_ready(&fstorage);

    // write
		
		
		
    rc = nrf_fstorage_write(&fstorage, fstorage.start_addr,fstorage_data_write_temp, sizeof(fstorage_data_write_temp), NULL);
			//rc = nrf_fstorage_write(&fstorage, fstorage.start_addr+offset,fstorage_data_write_temp, sizeof(fstorage_data_write_temp), NULL);
    if (NRF_SUCCESS != rc)
    {
        NRF_LOG_INFO("nrf_fstorage_write Failed, offset=%d, len=%d, rc=0x%x", offset, len, rc);
    }
    return rc;
}
#endif
#if 0
uint32_t fstorage_data_write(uint32_t offset, void const * p_src, uint32_t len)
{
    ret_code_t rc = NRF_SUCCESS;

    VERIFY_PARAM_NOT_NULL(p_src);
    VERIFY_FALSE((len + offset > sizeof(fstorage_data_write_temp)), NRF_ERROR_INVALID_PARAM);
		
	
		memcpy(fstorage_data_write_temp + offset,p_src,len);
    // prepare data
    rc = nrf_fstorage_read(&fstorage, fstorage.start_addr+offset,fstorage_data_write_temp+offset, sizeof(fstorage_data_write_temp));
    if (NRF_SUCCESS != rc)
    {
        NRF_LOG_INFO("nrf_fstorage_read fstorage init data Failed, 0x%x", rc);
    }
//    memcpy(fstorage_data_write_temp + offset, p_src, len);

    // erase
    //rc = nrf_fstorage_erase(&fstorage, fstorage.start_addr, 1, NULL);
	
//		uint8_t i;
//		i++;
		//if(i == 1){
			//rc = nrf_fstorage_erase(&fstorage, fstorage.start_addr, 1, NULL);
			rc = nrf_fstorage_erase(&fstorage, fstorage.start_addr+offset, 1, NULL);
			//NRF_LOG_INFO("nrf_fstorage_erase couter: %d", i);
			if (NRF_SUCCESS != rc)
			{

					NRF_LOG_INFO("nrf_fstorage_erase Failed, rc=0x%x", rc);
			}
		//}

    //wait_for_flash_ready(&fstorage);

    // write
		NRF_LOG_INFO("fstorage_data_write_temp:",(fstorage_data_write_temp));
		NRF_LOG_HEXDUMP_INFO(fstorage_data_write_temp, len);

    rc = nrf_fstorage_write(&fstorage, fstorage.start_addr,fstorage_data_write_temp, sizeof(fstorage_data_write_temp), NULL);
		//rc = nrf_fstorage_write(&fstorage, fstorage.start_addr+offset,fstorage_data_write_temp, sizeof(fstorage_data_write_temp), NULL);
    if (NRF_SUCCESS != rc)
    {
        NRF_LOG_INFO("nrf_fstorage_write Failed, offset=%d, len=%d, rc=0x%x", offset, len, rc);
    }
    return rc;
}
#endif
uint32_t fstorage_data_erase(uint32_t offset)

{
    ret_code_t rc = NRF_SUCCESS;

		rc = nrf_fstorage_erase(&fstorage, fstorage.start_addr+offset, 1, NULL);

    if (NRF_SUCCESS != rc)

    {

        NRF_LOG_INFO("nrf_fstorage_erase Failed, rc=0x%x", rc);

        //goto WRITE_OUT;

    }

    return rc;

}
int storageReadData(uint8_t *outBuf, uint32_t word_count, uint32_t offset)

{

    uint32_t err_code;

		//NRF_LOG_INFO("storageReadData");
    if (NULL == outBuf)
        return 0;
    if ((offset >= 1024) || (word_count + offset > 1024))
        return 0;
		if(word_count == 17){
		//NRF_LOG_INFO("VIN-17");
			err_code = fstorage_data_read(offset, outBuf, word_count);

			//NRF_LOG_HEXDUMP_INFO(outBuf, word_count);
		}
		if(word_count == 48){
			//NRF_LOG_INFO("CMPK-48");
			err_code = fstorage_data_read(offset, outBuf, word_count);
			//NRF_LOG_HEXDUMP_INFO(outBuf, word_count);

		}
    if (NRF_SUCCESS != err_code)
    {
			NRF_LOG_INFO("error:storageReadData function");
        return 0;
    }
    return 0;
}



int storageWriteData( unsigned char *inBuf, unsigned int word_count,unsigned int offset){
    uint32_t err_code;
    //NRF_LOG_INFO("storageWriteData");
    if (NULL == inBuf)
        return 0;

    if ((offset >= 1024) || (word_count + offset > 1024))
        return 0;

		if(word_count == 48){
		  //NRF_LOG_INFO("CMPK-48");
			err_code = fstorage_data_write(offset, inBuf, word_count);
	}

		if(word_count == 17){
		  //NRF_LOG_INFO("VIN-17");
			err_code = fstorage_data_write(offset, inBuf, word_count);
	}

    if (NRF_SUCCESS != err_code)
    {
			NRF_LOG_INFO("error:storageWriteData function");
        return 0;
    }
    return 0;
}
