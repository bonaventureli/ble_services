/**
 * Copyright (c) 2016 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** @file
 *
 * @brief fstorage file.
 *
 * This example showcases fstorage usage.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf_fstorage.h"
//#include "FreeRTOS.h"
//#include "semphr.h"

#ifdef SOFTDEVICE_PRESENT
#include "nrf_fstorage_sd.h"
#else
#include "nrf_fstorage_nvmc.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define FSTORAGE_PAGE_SIZES         (4096)
#define FSTORAGE_DATA_SIZE_IN_BYTES (256)  
#define FSTORAGE_DATA_ADDR_START    (0x79000)

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);

static SemaphoreHandle_t m_fs_semaphore = NULL;
NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage_data) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = FSTORAGE_DATA_ADDR_START,
    .end_addr   = (FSTORAGE_DATA_ADDR_START + FSTORAGE_PAGE_SIZES - 1),
};

__ALIGN(4) static uint8_t fstorage_data_write_temp[FSTORAGE_DATA_SIZE_IN_BYTES];

/**@brief   Helper function to obtain the last address on the last page of the on-chip flash that
 *          can be used to write user data.
 */
static uint32_t nrf5_flash_end_addr_get()
{
    uint32_t const bootloader_addr = NRF_UICR->NRFFW[0];
    uint32_t const page_sz         = NRF_FICR->CODEPAGESIZE;
    uint32_t const code_sz         = NRF_FICR->CODESIZE;

    return (bootloader_addr != 0xFFFFFFFF ?
            bootloader_addr : (code_sz * page_sz));
}

/**@brief   Sleep until an event is received. */
static void power_manage(void)
{
#ifdef SOFTDEVICE_PRESENT
    (void) sd_app_evt_wait();
#else
    __WFE();
#endif
}

static uint32_t fstorage_data_lock_init(void)
{
    if (NULL != m_fs_semaphore)
        return NRF_SUCCESS;
        
    m_fs_semaphore = xSemaphoreCreateMutex();//xSemaphoreCreateCounting(1, 1);
    if(NULL == m_fs_semaphore)
    {
        NRF_LOG_ERROR("m_fs_semaphore Failed!");
        return NRF_ERROR_RESOURCES;
    }

    return NRF_SUCCESS;
}

static uint32_t fstorage_data_lock_uninit(void)
{
    if (NULL == m_fs_semaphore)
        return NRF_SUCCESS;

    vSemaphoreDelete(m_fs_semaphore);

    return NRF_SUCCESS;
}

static void fstorage_data_lock(void)
{
    BaseType_t yield_req = pdFALSE;

    if (__get_IPSR() != 0) // in ISR
    {
        UNUSED_RETURN_VALUE(xSemaphoreTakeFromISR(m_fs_semaphore, &yield_req));           
        portYIELD_FROM_ISR(yield_req);
    }
    else // in Thread
    {
        UNUSED_RETURN_VALUE(xSemaphoreTake(m_fs_semaphore, portMAX_DELAY));
    }
}

static void fstorage_data_unlock(void)
{
    BaseType_t yield_req = pdFALSE;
        
    if (__get_IPSR() != 0) // in ISR
    {
        // The returned value may be safely ignored, if error is returned it only means that
        // the semaphore is already given (raised).
        UNUSED_VARIABLE(xSemaphoreGiveFromISR(m_fs_semaphore, &yield_req));
        portYIELD_FROM_ISR(yield_req);
    }
    else // in Thread
    {
        UNUSED_VARIABLE(pdTRUE != xSemaphoreGive(m_fs_semaphore));
    }
}

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("fstorage opera Failed, id=%d.", p_evt->id);
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            //NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
            //             p_evt->len, p_evt->addr);
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            //NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
            //             p_evt->len, p_evt->addr);
        } break;

        default:
            break;
    }
}
#if 0
static void print_flash_info(nrf_fstorage_t * p_fstorage)
{
    NRF_LOG_INFO("========| flash info |========");
    NRF_LOG_INFO("erase unit: \t%d bytes",      p_fstorage->p_flash_info->erase_unit);
    NRF_LOG_INFO("program unit: \t%d bytes",    p_fstorage->p_flash_info->program_unit);
    NRF_LOG_INFO("==============================");
}
#endif
static void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)
{
    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(p_fstorage))
    {
        power_manage();
    }
}

uint32_t fstorage_data_init(void)
{
    ret_code_t rc;
    nrf_fstorage_api_t * p_fs_api;

#ifdef SOFTDEVICE_PRESENT
    /* Initialize an fstorage instance using the nrf_fstorage_sd backend.
     * nrf_fstorage_sd uses the SoftDevice to write to flash. This implementation can safely be
     * used whenever there is a SoftDevice, regardless of its status (enabled/disabled). */
    p_fs_api = &nrf_fstorage_sd;
#else
    /* Initialize an fstorage instance using the nrf_fstorage_nvmc backend.
     * nrf_fstorage_nvmc uses the NVMC peripheral. This implementation can be used when the
     * SoftDevice is disabled or not present.
     *
     * Using this implementation when the SoftDevice is enabled results in a hardfault. */
    p_fs_api = &nrf_fstorage_nvmc;
#endif

    rc = nrf_fstorage_init(&fstorage_data, p_fs_api, NULL);
    if (NRF_SUCCESS != rc)
    {
        NRF_LOG_ERROR("nrf_fstorage_init Failed, 0x%x", rc);
        return rc;
    }

    //print_flash_info(&fstorage_data);

    /* It is possible to set the start and end addresses of an fstorage instance at runtime.
     * They can be set multiple times, should it be needed. The helper function below can
     * be used to determine the last address on the last page of flash memory available to
     * store data. */
    NRF_LOG_INFO("nrf5_flash_end_addr=0x%x", nrf5_flash_end_addr_get());

    return fstorage_data_lock_init();
}

uint32_t fstorage_data_uinit(void)
{
    ret_code_t rc;

    rc = nrf_fstorage_uninit(&fstorage_data, NULL);
    if (NRF_SUCCESS != rc)
    {
        NRF_LOG_ERROR("nrf_fstorage_uninit Failed, 0x%x", rc);
        return rc;
    }

    return fstorage_data_lock_uninit();
}

uint32_t fstorage_data_write(uint32_t offset, void const * p_src, uint32_t len)
{
    ret_code_t rc = NRF_SUCCESS;
	  
    //memcpy(fstorage_data_write_temp + offset,p_src,len);
	  //return rc;
	
    VERIFY_PARAM_NOT_NULL(p_src);
    VERIFY_FALSE((len + offset > sizeof(fstorage_data_write_temp)), NRF_ERROR_INVALID_PARAM);

    fstorage_data_lock();

    // prepare data
    rc = nrf_fstorage_read(&fstorage_data, fstorage_data.start_addr,
                           fstorage_data_write_temp, sizeof(fstorage_data_write_temp));
    if (NRF_SUCCESS != rc)
    {
        NRF_LOG_ERROR("nrf_fstorage_read fstorage init data Failed, 0x%x", rc);
        goto WRITE_OUT;
    }
    
    memcpy(fstorage_data_write_temp + offset, p_src, len);

    // erase
    rc = nrf_fstorage_erase(&fstorage_data, fstorage_data.start_addr, 1, NULL);
    if (NRF_SUCCESS != rc)
    {
        NRF_LOG_ERROR("nrf_fstorage_erase Failed, rc=0x%x", rc);
        goto WRITE_OUT;
    }
    wait_for_flash_ready(&fstorage_data);
    
    // write
    rc = nrf_fstorage_write(&fstorage_data, fstorage_data.start_addr,
                            fstorage_data_write_temp, sizeof(fstorage_data_write_temp), NULL);
    if (NRF_SUCCESS != rc)
    {
        NRF_LOG_ERROR("nrf_fstorage_write Failed, offset=%d, len=%d, rc=0x%x", offset, len, rc);
        goto WRITE_OUT;
    }

    wait_for_flash_ready(&fstorage_data);


WRITE_OUT:
    fstorage_data_unlock();

    return rc;
}

uint32_t fstorage_data_read(uint32_t offset, uint8_t * p_dest, uint32_t len)
{
    ret_code_t rc = NRF_SUCCESS;
    VERIFY_PARAM_NOT_NULL(p_dest);
    VERIFY_FALSE((len + offset > sizeof(fstorage_data_write_temp)), NRF_ERROR_INVALID_PARAM);

#if 0
    fstorage_data_lock();

    // prepare data
    rc = nrf_fstorage_read(&fstorage_data, fstorage_data.start_addr,
                           fstorage_data_write_temp, sizeof(fstorage_data_write_temp));
    if (NRF_SUCCESS != rc)
    {
        NRF_LOG_ERROR("nrf_fstorage_read fstorage init data Failed, 0x%x", rc);
        goto READ_OUT;
    }

    memcpy(p_dest, fstorage_data_write_temp + offset, len);

READ_OUT:

    fstorage_data_unlock();
#else
    memcpy(p_dest,(void*) (fstorage_data.start_addr + offset), len);
#endif

    return rc;
}

/**
 * @}
 */
