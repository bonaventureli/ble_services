/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
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
/** @example examples/ble_peripheral/ble_app_hrs/main.c
 *
 * @brief Heart Rate Service Sample Application main file.
 *
 * This file contains the source code for a sample application using the Heart Rate service
 * (and also Battery and Device Information services). This application uses the
 * @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "app_timer.h"
#include "app_uart.h"
#include "bsp_btn_ble.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "fds.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_lesc.h"
#include "nrf_ble_qwr.h"
#include "ble_conn_state.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "digital_key_api.h"
#include "ikcmdif.h"
#include "nrf_drv_rng.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif


#define DEVICE_NAME                         "GACVK" 
//#define DEVICE_NAME                         "xxx"                            /**< Name of device. Will be included in the advertising data. */
//#define DEVICE_NAME                         "Nordic_HRM"                            /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                   "NordicSemiconductor"                   /**< Manufacturer. Will be passed to Device Information Service. */

#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN  //add lifei 2018/11/2

#define APP_ADV_INTERVAL                    300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION                    18000                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO               3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define BATTERY_LEVEL_MEAS_INTERVAL         APP_TIMER_TICKS(2000)                   /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL                   81                                      /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                   100                                     /**< Maximum simulated 7battery level. */
#define BATTERY_LEVEL_INCREMENT             1                                       /**< Increment between each simulated battery level measurement. */

#define HEART_RATE_MEAS_INTERVAL            APP_TIMER_TICKS(1000)                   /**< Heart rate measurement interval (ticks). */
#define MIN_HEART_RATE                      140                                     /**< Minimum heart rate as returned by the simulated measurement function. */
#define MAX_HEART_RATE                      300                                     /**< Maximum heart rate as returned by the simulated measurement function. */
#define HEART_RATE_INCREMENT                10                                      /**< Value by which the heart rate is incremented/decremented for each call to the simulated measurement function. */

#define RR_INTERVAL_INTERVAL                APP_TIMER_TICKS(300)                    /**< RR interval interval (ticks). */
#define MIN_RR_INTERVAL                     100                                     /**< Minimum RR interval as returned by the simulated measurement function. */
#define MAX_RR_INTERVAL                     500                                     /**< Maximum RR interval as returned by the simulated measurement function. */
#define RR_INTERVAL_INCREMENT               1                                       /**< Value by which the RR interval is incremented/decremented for each call to the simulated measurement function. */

#define SENSOR_CONTACT_DETECTED_INTERVAL    APP_TIMER_TICKS(5000)                   /**< Sensor Contact Detected toggle interval (ticks). */

#define MIN_CONN_INTERVAL                   MSEC_TO_UNITS(400, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(650, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                       0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY      APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY       APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT        3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define LESC_DEBUG_MODE                     0                                       /**< Set to 1 to use LESC debug keys, allows you to use a sniffer to inspect traffic. */

#define SEC_PARAM_BOND                      1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                      0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      0                                       /**< LE Secure Connections enabled. */
#define SEC_PARAM_KEYPRESS                  0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                       0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */


BLE_HRS_DEF(m_hrs);                                                 /**< Heart rate service instance. */
BLE_BAS_DEF(m_bas);                                                 /**< Structure used to identify the battery service. */
NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                 /**< Advertising module instance. */
APP_TIMER_DEF(m_battery_timer_id);                                  /**< Battery timer. */
APP_TIMER_DEF(m_heart_rate_timer_id);                               /**< Heart rate measurement timer. */
APP_TIMER_DEF(m_rr_interval_timer_id);                              /**< RR interval timer. */
APP_TIMER_DEF(m_sensor_contact_timer_id);                           /**< Sensor contact detected timer. */

static uint16_t m_conn_handle         = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static bool     m_rr_interval_enabled = true;                       /**< Flag for enabling and disabling the registration of new RR interval measurements (the purpose of disabling this is just to test sending HRM without RR interval data. */

static sensorsim_cfg_t   m_battery_sim_cfg;                         /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t m_battery_sim_state;                       /**< Battery Level sensor simulator state. */
static sensorsim_cfg_t   m_heart_rate_sim_cfg;                      /**< Heart Rate sensor simulator configuration. */
static sensorsim_state_t m_heart_rate_sim_state;                    /**< Heart Rate sensor simulator state. */
static sensorsim_cfg_t   m_rr_interval_sim_cfg;                     /**< RR Interval sensor simulator configuration. */
static sensorsim_state_t m_rr_interval_sim_state;                   /**< RR Interval sensor simulator state. */

static ble_uuid_t m_adv_uuids[] =                                   /**< Universally unique service identifiers. */
{
    {BLE_UUID_HEART_RATE_SERVICE,           BLE_UUID_TYPE_BLE},
    {BLE_UUID_BATTERY_SERVICE,              BLE_UUID_TYPE_BLE},
    {BLE_UUID_DEVICE_INFORMATION_SERVICE,   BLE_UUID_TYPE_BLE},
		//{SDK_UUID_SERVICE, NUS_SERVICE_UUID_TYPE},//add lifei 2018/11/2
};

uint8_t gActive;
uint8_t gReturnInfo[29];
unsigned int gReturnInfoLen;
uint8_t gReturnSession[131];
unsigned int gReturnSessionLen;

//int ingeek_pull_info(unsigned char *output, unsigned int* olen);

uint8_t VIN_data[17];
uint8_t CMPK_data[48];
uint8_t VIN_CMPK_data[70];//65

uint8_t Rand_data[32]={
0xB1,0xB1,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBF,
0xB1,0xB1,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBF,
0xB1,0xB1,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBF,
0xB1,0xB1
};

//#include "nrf_fstorage.h"
//#include "fds.h"
#ifdef SOFTDEVICE_PRESENT
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_fstorage_sd.h"
#else
#include "nrf_drv_clock.h"
#include "nrf_fstorage_nvmc.h"
#endif
#define FSTORAGE_PAGE_SIZES         (4096)
#define FSTORAGE_DATA_ADDR_START    (0x79000)

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

static int storageWriteData( uint8_t *inBuf, uint32_t word_count,uint32_t offset);
static int storageReadData(uint8_t *outBuf, uint32_t word_count, uint32_t offset);
void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage);

int read_CB1(unsigned char *out, unsigned int rlen, unsigned int offset)
{
	NRF_LOG_INFO("  read_CB1  ");
	NRF_LOG_HEXDUMP_INFO(out, rlen);	

	#if USE_DATA_FLASH
	SdkRead(out,rlen,offset);
	#else
		//memcpy(out,Callback_data+offset,rlen);
	#endif
	if(rlen == 17){
		NRF_LOG_INFO("VIN");
		//storageReadData(VIN_data,rlen,0);
		//memcpy(out,VIN_data,rlen);
	 //nrf_fstorage_read(&fstorage, fstorage.start_addr+offset,out, rlen);
	 memcpy(out,fstorage_data_write_temp,sizeof(fstorage_data_write_temp));
	 NRF_LOG_HEXDUMP_INFO(out, sizeof(rlen));
	}
	if(rlen == 48){
		NRF_LOG_INFO("CMPK");
		//storageReadData(CMPK_data,rlen,0);
		memcpy(out,CMPK_data,rlen);
	 //nrf_fstorage_read(&fstorage, fstorage.start_addr+offset,out, rlen);
	 //memcpy(out,fstorage_data_write_temp,sizeof(fstorage_data_write_temp));
	 NRF_LOG_HEXDUMP_INFO(out, sizeof(rlen));
	}
	return 0;
}
/*
* Function:    read_CB1
* Description:        
* Parameter:   None
* Return:      int
* auther: lifei 
* change time£º2018/8/31
*/
int write_CB1(unsigned char *in, unsigned int wlen, unsigned int offset){
	NRF_LOG_INFO("  write_CB1  ");
	NRF_LOG_HEXDUMP_INFO(in, wlen);	
	#if USE_DATA_FLASH
		SdkWrite(in,wlen,offset);
	#else
		//memcpy(Callback_data+offset,in,wlen);
	#endif
	#if 1
	if(wlen == 48){
	NRF_LOG_INFO("CMPK-48");
	//storageWriteData(in, wlen,offset);
	memcpy(CMPK_data,in,wlen);
	}
		if(wlen == 17){
	NRF_LOG_INFO("VIN-17");
	memcpy(VIN_data,in,wlen);
	//storageWriteData(VIN_data, wlen,offset);
	}
	#endif
	
	return 0;
}
/*
* Function:    read_CB1
* Description:        
* Parameter:   None
* Return:      int
* auther: lifei 
* change time£º2018/8/31
*/
int Rand_CB1(void *p_rng, unsigned char *rand, unsigned int randlen){
	
	p_rng = 0;
	
	if(p_rng == 0){
	}
	NRF_LOG_INFO("  Rand_CB1  ");
	
	if(randlen == 8){
		memcpy(rand,Rand_data,8);
		NRF_LOG_INFO("RANDDATA-8");
	}
	if(randlen == 32){
		memcpy(rand,Rand_data,32);
		NRF_LOG_INFO("RANDDATA-32");
	}
	NRF_LOG_HEXDUMP_INFO(rand, randlen);
	return 0;

}
void g_printcb(const char *fmt, int len)
{
//	unsigned char len_t;
//	len_t = (unsigned char)len;
//	Uart3Sent(fmt,len_t);
}

/** @brief Function for getting vector of random numbers.
 *
 * @param[out] p_buff                               Pointer to unit8_t buffer for storing the bytes.
 * @param[in]  length                               Number of bytes to take from pool and place in p_buff.
 *
 * @retval     Number of bytes actually placed in p_buff.
 */ 
static int ikif_random_vector_generate(void * seed ,unsigned char * p_buff, uint32_t size)
{
    uint8_t available;
    uint32_t err_code;
    //err_code = nrf_drv_rng_bytes_available(&available);
		nrf_drv_rng_bytes_available(&available);
    //APP_ERROR_CHECK(err_code);
    uint8_t length = (size<available) ? size : available;
    err_code = nrf_drv_rng_rand(p_buff,length);
    //APP_ERROR_CHECK(err_code);
	
	seed = 0;
	
	if(seed == 0){
	}
	
	if(size == 8){
		
		NRF_LOG_INFO("RANDDATA-8");
	}
	if(size == 32){
		
		NRF_LOG_INFO("RANDDATA-32");
	}
	NRF_LOG_HEXDUMP_INFO(p_buff, length);
    return 0;
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}
uint32_t ble_send_notify(uint16_t uuid, uint8_t * data, uint16_t length)
{
      ble_send(&m_hrs, uuid, data, length);
      return 1;
}

/**@brief Function for starting advertising.
 */
#if 0
static uint8_t  initial_info[29]={0x0A,0x11,0x4B,0x4D,0x4C,0x38,0x46,0x30,0x30,0x39,0x31,0x43,0x43,0x32,0x34,0x46,0x44,0x41,0x31,0x12,0x08,0xB1,0xB1,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8};
unsigned int initial_infoLen=29;
#endif

void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        ret_code_t err_code;

        err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
		#if 0
			//if (gActive == 1){
			uint8_t status = 0;
			ble_send_notify(BLE_UUID_DIGITALKET_STATUS_CHAR, &status, 1);
		  ble_send_notify(BLE_UUID_DIGITALKET_SESSION_CHAR, initial_info, initial_infoLen);
		  NRF_LOG_HEXDUMP_INFO(initial_info, initial_infoLen);
				//gActive = 0;
			//}
		#endif
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        default:
            break;
    }
}


/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
#if 1
static void battery_level_update(void)
{
    ret_code_t err_code;
    uint8_t  battery_level;

    battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);

    err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}
#endif

/**@brief Function for handling the Heart rate measurement timer timeout.
 *
 * @details This function will be called each time the heart rate measurement timer expires.
 *          It will exclude RR Interval data from every third measurement.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void heart_rate_meas_timeout_handler(void * p_context)
{
    static uint32_t cnt = 0;
    ret_code_t      err_code;
    uint16_t        heart_rate;

    UNUSED_PARAMETER(p_context);

    heart_rate = (uint16_t)sensorsim_measure(&m_heart_rate_sim_state, &m_heart_rate_sim_cfg);

    cnt++;
    err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, heart_rate);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }

    // Disable RR Interval recording every third heart rate measurement.
    // NOTE: An application will normally not do this. It is done here just for testing generation
    // of messages without RR Interval measurements.
    m_rr_interval_enabled = ((cnt % 3) != 0);
}


/**@brief Function for handling the RR interval timer timeout.
 *
 * @details This function will be called each time the RR interval timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void rr_interval_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    if (m_rr_interval_enabled)
    {
        uint16_t rr_interval;

        rr_interval = (uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                                                  &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
        rr_interval = (uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                                                  &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
        rr_interval = (uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                                                  &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
        rr_interval = (uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                                                  &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
        rr_interval = (uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                                                  &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
        rr_interval = (uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                                                  &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
    }
}


/**@brief Function for handling the Sensor Contact Detected timer timeout.
 *
 * @details This function will be called each time the Sensor Contact Detected timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
#if 1
static void sensor_contact_detected_timeout_handler(void * p_context)
{
    static bool sensor_contact_detected = false;

    UNUSED_PARAMETER(p_context);

    sensor_contact_detected = !sensor_contact_detected;
    ble_hrs_sensor_contact_detected_update(&m_hrs, sensor_contact_detected);
}
#endif

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
	#if 1
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
	#endif

    err_code = app_timer_create(&m_heart_rate_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                heart_rate_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_rr_interval_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                rr_interval_timeout_handler);
    APP_ERROR_CHECK(err_code);
#if 1
    err_code = app_timer_create(&m_sensor_contact_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                sensor_contact_detected_timeout_handler);
    APP_ERROR_CHECK(err_code);
#endif
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief GATT module event handler.
 */
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("GATT ATT MTU on connection 0x%x changed to %d.",
                     p_evt->conn_handle,
                     p_evt->params.att_mtu_effective);
    }

    ble_hrs_on_gatt_evt(&m_hrs, p_evt);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


uint8_t ble_set_info(ble_hrs_t * p_hrs, uint8_t * data, uint16_t length)
{
    ret_code_t         err_code = NRF_SUCCESS;
	  ble_gatts_value_t  gatts_value;
        // Initialize value struct.
        memset(&gatts_value, 0, sizeof(gatts_value));

        gatts_value.len     = length;
        gatts_value.offset  = 0;
        gatts_value.p_value = data;

				NRF_LOG_DEBUG("This is ble_set_info function\n");
        // Update database.
        err_code = sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID,
                                          p_hrs->info_handles.value_handle,
                                          &gatts_value);
        if (err_code == NRF_SUCCESS)
        {
            //NRF_LOG_INFO("Battery level has been updated: %d%%", battery_level)

            // Save new battery value.
            //p_bas->battery_level_last = battery_level;
        }
        else
        {
            NRF_LOG_DEBUG("Error ikble_set_se_info: 0x%08X", err_code)

            return err_code;
        }
	
	  return 0;
}

uint32_t ikble_set_se_info(uint8_t * data, uint16_t length)
{
      ble_set_info(&m_hrs, data, length);
      return 1;
}

void Handle_info(uint8_t *data, uint32_t data_len)
{
	unsigned int outlen;
	uint8_t *preply_data;
	uint8_t status;
	int ret;
	
	// One,push info
	NRF_LOG_INFO("before:ingeek_push_info");
	//NRF_LOG_HEXDUMP_INFO(data, data_len);
	//ingeek_push_info(data, data_len);
	
	ret = ingeek_push_info(data, data_len);
	if (ret != 0x00){
	NRF_LOG_INFO("error return %d :ingeek_push_info",ret);
	}
	
	NRF_LOG_INFO("after:ingeek_push_info");
	//NRF_LOG_HEXDUMP_INFO(data, data_len);
	
	

	//Two,Notify status
	status = ingeek_get_sec_status();
	ble_send_notify(BLE_UUID_DIGITALKET_STATUS_CHAR, &status, 1);
	
	//Three,pull info
	ingeek_pull_info(gReturnInfo, &gReturnInfoLen);
	//NRF_LOG_INFO("Handle_info:[After] ingeek_pull_info:status:ingeek_get_sec_status is %x. if 1,it is right\n",ingeek_get_sec_status());
	//Fore,Notify info
	ble_send_notify(BLE_UUID_DIGITALKET_INFO_CHAR, gReturnInfo, gReturnInfoLen);
	//ikble_set_se_info(gReturnInfo,gReturnInfoLen);
	
	

//		nrf_fstorage_read(&fstorage, fstorage.start_addr,fstorage_data_write_temp, sizeof(fstorage_data_write_temp));
//		nrf_fstorage_erase(&fstorage,fstorage.start_addr,1,NULL);
//		wait_for_flash_ready(&fstorage);
//		nrf_fstorage_write(&fstorage,fstorage.start_addr,CMPK_data,sizeof(CMPK_data),NULL);
		
		//nrf_fstorage_write(&fstorage,fstorage.start_addr,VIN_data,sizeof(VIN_data),NULL);

}
void Handle_auth(uint8_t *data, uint32_t data_len)
{
	uint8_t status;
	
	#if 0
	status = ingeek_get_sec_status();
	
	NRF_LOG_INFO("\nHandle_auth_function:[Before] ingeek_push_auth:status:ingeek_get_sec_status is %x. if 1,it is right\n",ingeek_get_sec_status());
	NRF_LOG_INFO("\nHandle_auth_function,return:ingeek_push_auth %x, if 0,it is right\n",ingeek_push_auth(data, data_len, (unsigned char*)1, (unsigned int*)1));
	NRF_LOG_INFO("\nHandle_auth_function:[After] ingeek_push_auth:status:ingeek_get_sec_status is %x. if 2,it is right\n",ingeek_get_sec_status());
	#endif
	ingeek_push_auth(data, data_len, (unsigned char*)1, (unsigned int*)1);
	status = ingeek_get_sec_status();
	if(status == 0x02)
	ble_send_notify(BLE_UUID_DIGITALKET_STATUS_CHAR, &status, 1);
	
	
	#if 0
	NRF_LOG_HEXDUMP_INFO(data, data_len);
	ingeek_push_auth(data, data_len, (unsigned char*)1, (unsigned int*)1);
	#endif
	
	#if 0
	
	ingeek_push_auth(data, data_len, (unsigned char*)1, (unsigned int*)1);
	status = ingeek_get_sec_status();
	NRF_LOG_INFO("\nHandle_auth_function:[After] ingeek_push_auth:status:ingeek_get_sec_status is %x. if 2,it is right\n",ingeek_get_sec_status());
	if(status == 0x02)
	ble_send_notify(BLE_UUID_DIGITALKET_STATUS_CHAR, &status, 1);
	#endif
}

void Handle_session(uint8_t *data, uint32_t data_len)
{
	unsigned int outlen;
	uint8_t *preply_data;
	uint8_t status;

	#if 1
	NRF_LOG_INFO("\nHandle_session_function:[Before] ingeek_push_session:status:ingeek_get_sec_status is %x. if 2,it is right\n",ingeek_get_sec_status());
	NRF_LOG_INFO("\nHandle_session_function,return:ingeek_push_session %d, if 0,it is right\n",ingeek_push_session(data, data_len, preply_data, &outlen));
	NRF_LOG_INFO("\nHandle_session_function:[After] ingeek_push_session:status:ingeek_get_sec_status is %x. if 3,it is right\n",ingeek_get_sec_status());
	
	NRF_LOG_INFO("\nHandle_session is ok ,ingeek_push_session,preply_data:\n");
	#endif

	//ingeek_push_session(data, data_len, gReturnSession, &gReturnSessionLen);
	status = ingeek_get_sec_status();
	ble_send_notify(BLE_UUID_DIGITALKET_STATUS_CHAR, &status, 1);
	ble_send_notify(BLE_UUID_DIGITALKET_SESSION_CHAR, gReturnSession, gReturnSessionLen);
	
	#if 0
	ingeek_push_session(data, data_len, preply_data, &outlen);
	NRF_LOG_HEXDUMP_INFO(preply_data, outlen);
	#endif
	
	#if 0
	status = ingeek_get_sec_status();
	ble_send_notify(BLE_UUID_DIGITALKET_STATUS_CHAR, &status, 1);
	ble_send_notify(BLE_UUID_DIGITALKET_SESSION_CHAR, preply_data, outlen);
	#endif

}
extern T_CarCMD CarCMD;
void Handle_cmd(uint8_t *data, uint32_t data_len)
{
	uint8_t *preply_data;
	unsigned int outlen;
	

	DK_Cmd_Meg struct_cmd;
	uint8_t cmd;

	#if 0
	NRF_LOG_HEXDUMP_INFO(data, data_len);
	NRF_LOG_INFO("\nHandle_cmd_function:[Before] ingeek_command_input_action:status:ingeek_get_sec_status is %x. if 3,it is right\n",ingeek_get_sec_status());
	NRF_LOG_INFO("\nHandle_cmd_function,return:ingeek_command_input_action %x, if 0,it is right\n",ingeek_command_input_action(data, data_len, &struct_cmd));
	NRF_LOG_INFO("\nHandle_cmd_function:[After] ingeek_command_input_action:status:ingeek_get_sec_status is %x. if 3,it is right\n",ingeek_get_sec_status());
	#endif
	int i ;
	for(i=0;i<1000;i++);
	ingeek_command_input_action(data, data_len, &struct_cmd);
	for(i=0;i<1000;i++);
	NRF_LOG_INFO("\nHandle_cmd_command: %d \n",(uint8_t)(struct_cmd.command));
	ikcmdSendUart(struct_cmd.command);
	
	
	#if 0
	ingeek_command_input_action(data, data_len, &struct_cmd);
	#endif
	
	#if 0
	NRF_LOG_INFO("\nHandle_cmd_function,return:ingeek_command_output_action %x, if 0,it is right\n",ingeek_command_output_action(&struct_cmd,preply_data, &outlen));
	NRF_LOG_INFO("\nHandle_cmd is ok ,ingeek_command_input_action,preply_data:\n");
	
	#endif
	
	#if 1
	ingeek_command_output_action(&struct_cmd,preply_data, &outlen);
	#endif
	
	ble_send_notify(BLE_UUID_DIGITALKET_CMD_CHAR, preply_data, outlen);

}
#define SESSIONMAXBIT 112
#define INFOCHARMAXBIT 29
#define DK_REPLAY_MAXBIT 128
void ble_hrs_evt_handler (ble_hrs_t * p_hrs, ble_hrs_evt_t * p_evt){

	uint8_t status = 0;
	//NRF_LOG_INFO("ble_hrs_evt_handler function==========");
	switch(p_evt->evt_type)
	{
		case BLE_HRS_EVT_NOTIFICATION_ENABLED:
		{
			NRF_LOG_INFO("BLE_EVT_NOTIFICATION_ENABLED");
			break;
		}
		case BLE_HRS_EVT_NOTIFICATION_DISABLED:
		{
			NRF_LOG_INFO("BLE_EVT_NOTIFICATION_DISABLED");

			break;
		}
		case BLE_STATUS_EVT_NOTIFICATION_ENABLED:
		{
			NRF_LOG_INFO("BLE_STATUS_EVT_NOTIFICATION_ENABLED");
				//status = ingeek_get_sec_status();
				//ble_send_notify(BLE_UUID_DIGITALKET_STATUS_CHAR, &status, 1);
			break;
		}
		case BLE_STATUS_EVT_NOTIFICATION_DISABLED:
		{
			NRF_LOG_INFO("BLE_STATUS_EVT_NOTIFICATION_DISABLED");

			break;
		}
		case BLE_INFO_EVT_NOTIFICATION_ENABLED:
		{
			NRF_LOG_INFO("BLE_INFO_EVT_NOTIFICATION_ENABLED");
			//ble_send_notify(BLE_UUID_DIGITALKET_INFO_CHAR, gReturnInfo, gReturnInfoLen);
			break;
		}
		case BLE_INFO_EVT_NOTIFICATION_DISABLED:
		{
			NRF_LOG_INFO("BLE_INFO_EVT_NOTIFICATION_DISABLED");

			break;
		}
		case BLE_SESSION_EVT_NOTIFICATION_ENABLED:
		{
			NRF_LOG_INFO("BLE_SESSION_EVT_NOTIFICATION_ENABLED");
			ble_send_notify(BLE_UUID_DIGITALKET_SESSION_CHAR, gReturnSession, gReturnSessionLen);
			break;
		}
		case BLE_SESSION_EVT_NOTIFICATION_DISABLED:
		{
			NRF_LOG_INFO("BLE_SESSION_EVT_NOTIFICATION_DISABLED");

			break;
		}
		case BLE_CMD_EVT_NOTIFICATION_ENABLED:
		{
			NRF_LOG_INFO("BLE_CMD_EVT_NOTIFICATION_ENABLED");
			break;
		}
		case BLE_CMD_EVT_NOTIFICATION_DISABLED:
		{
			NRF_LOG_INFO("BLE_CMD_EVT_NOTIFICATION_DISABLED");

			break;
		}		
		case BLE_DIGITAKKEY_EVT_INFO:
		{
			NRF_LOG_INFO("case BLE_DIGITAKKEY_EVT_INFO");
			Handle_info((uint8_t *)p_evt->params.rx_data.p_data, (uint32_t) p_evt->params.rx_data.length);
			break;
		}
		case BLE_DIGITAKKEY_EVT_AUTH:
		{
			NRF_LOG_INFO("case BLE_DIGITAKKEY_EVT_AUTH");
			//NRF_LOG_HEXDUMP_INFO((uint8_t *)p_evt->params.rx_data.p_data, (uint16_t)p_evt->params.rx_data.length);	
			Handle_auth((uint8_t *)p_evt->params.rx_data.p_data, (uint32_t) p_evt->params.rx_data.length);
			break;
		}
		case BLE_DIGITAKKEY_EVT_SESSION:
		{
			NRF_LOG_INFO("case BLE_DIGITAKKEY_EVT_SESSION");
			Handle_session((uint8_t *)p_evt->params.rx_data.p_data, (uint32_t) p_evt->params.rx_data.length);
			break;
		}
		case BLE_DIGITAKKEY_EVT_CMD:
		{
			NRF_LOG_INFO("case BLE_DIGITAKKEY_EVT_CMD");
			Handle_cmd((uint8_t *)p_evt->params.rx_data.p_data, (uint32_t) p_evt->params.rx_data.length);
			break;
		}
		
		case BLE_DIGITAKKEY_EVT_RSSI:
		{
			NRF_LOG_INFO("case BLE_DIGITAKKEY_EVT_RSSI");
			NRF_LOG_HEXDUMP_INFO((uint8_t *)p_evt->params.rx_data.p_data, (uint16_t)p_evt->params.rx_data.length);	
			break;
		}
		#if 0
		case BLE_DIGITAKKEY_EVT_VERSION:
		{
			NRF_LOG_INFO("This is BLE_DIGITAKKEY_EVT_VERSION function add lifei 2018/11/2 17:52.");
			NRF_LOG_HEXDUMP_INFO((uint8_t *)p_evt->params.rx_data.p_data, (uint16_t)p_evt->params.rx_data.length);	
			//Handle_cmd((uint8_t *)p_evt->params.rx_data.p_data, (uint32_t) p_evt->params.rx_data.length);
			break;
		}
		
		default:
			NRF_LOG_INFO("This is ble_hrs_evt_handler function add lifei 2018/11/2 17:18.");
			break;
		#endif
	}
	
}
/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    ble_hrs_init_t     hrs_init;
    ble_bas_init_t     bas_init;
    ble_dis_init_t     dis_init;
    nrf_ble_qwr_init_t qwr_init = {0};
    uint8_t            body_sensor_location;
		
		

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Heart Rate Service.
    body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_FINGER;

    memset(&hrs_init, 0, sizeof(hrs_init));

    hrs_init.evt_handler                 = ble_hrs_evt_handler;//NULL; //change lifei 2018/11/2
    hrs_init.is_sensor_contact_supported = true;
    hrs_init.p_body_sensor_location      = &body_sensor_location;

    // Here the sec level for the Heart Rate Service can be changed/increased.
    hrs_init.hrm_cccd_wr_sec = SEC_OPEN;
    hrs_init.bsl_rd_sec      = SEC_OPEN;

    err_code = ble_hrs_init(&m_hrs, &hrs_init);
    APP_ERROR_CHECK(err_code);

		#if 1
    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    // Here the sec level for the Battery Service can be changed/increased.
    bas_init.bl_rd_sec        = SEC_OPEN;
    bas_init.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

		#endif
    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);

    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
		
}


/**@brief Function for initializing the sensor simulators.
 */
static void sensor_simulator_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;

    sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);

    m_heart_rate_sim_cfg.min          = MIN_HEART_RATE;
    m_heart_rate_sim_cfg.max          = MAX_HEART_RATE;
    m_heart_rate_sim_cfg.incr         = HEART_RATE_INCREMENT;
    m_heart_rate_sim_cfg.start_at_max = false;

    sensorsim_init(&m_heart_rate_sim_state, &m_heart_rate_sim_cfg);

    m_rr_interval_sim_cfg.min          = MIN_RR_INTERVAL;
    m_rr_interval_sim_cfg.max          = MAX_RR_INTERVAL;
    m_rr_interval_sim_cfg.incr         = RR_INTERVAL_INCREMENT;
    m_rr_interval_sim_cfg.start_at_max = false;

    sensorsim_init(&m_rr_interval_sim_state, &m_rr_interval_sim_cfg);
}


/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    ret_code_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_rr_interval_timer_id, RR_INTERVAL_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
#if 1
    err_code = app_timer_start(m_sensor_contact_timer_id, SENSOR_CONTACT_DETECTED_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
#endif
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = m_hrs.hrm_handles.cccd_handle;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}

void Handle_active()
{
	#if 1
	NRF_LOG_INFO("Handle_active function");
	uint8_t *preply_data;
	unsigned int outlen;
//	preply_data = gDataload;
	
	//ingeek_se_final();
	//ingeek_se_init();
	uint8_t status;
	
	status = ingeek_get_sec_status();
	ble_send_notify(BLE_UUID_DIGITALKET_STATUS_CHAR, &status, 1);
	
		if(ingeek_pull_info(gReturnInfo, &gReturnInfoLen) != SUCCESS){
		//if(ingeek_pull_info(preply_data, &outlen) != SUCCESS){
		
		NRF_LOG_INFO("Handle_active:ingeek_pull_info Not Success");
		return;
	}
	else{
		if(gReturnInfoLen != 29){
			NRF_LOG_INFO("Handle_active:outlen != 29");
			return;
		}
		else{
			NRF_LOG_INFO("Handle_active:ingeek_pull_info Success");
			ble_send_notify(BLE_UUID_DIGITALKET_INFO_CHAR, gReturnInfo, gReturnInfoLen);
		}
	}
	#endif
}

void Handle_broadcast()
{
	int ret;
	NRF_LOG_INFO("Handle_broadcast function==========");
	ret = ingeek_get_sec_status();
	if(ret == 0xFF){
		NRF_LOG_INFO("status %x",ret);
		//Handle_noactive();
		//NRF_LOG_INFO("status %x",ret);
		return;
	}
	#if 0
	if((ret >= 0)&&(ret < 0xFF)){
		NRF_LOG_INFO("status %x",ret);
		Handle_active();
	}
	#endif
	if((ret == 0) || (ret == 0x01)){
		NRF_LOG_INFO("status %x",ret);
		Handle_active();
	}
}
void Handle_disconnect()
{
	int ret;
	
	#if 0
	if((ret == 0x00)||(ret == 0x01)){
		NRF_LOG_INFO("status %x",ret);
	}
	else{
	ingeek_se_final();
		NRF_LOG_INFO("ingeek_se_final");
	}
	#endif
	
	ingeek_se_final();
	ret = ingeek_get_sec_status();
	NRF_LOG_INFO("status %x",ret);
	NRF_LOG_INFO("Handle_disconnect function==========");
	
	return;
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
						Handle_broadcast();
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected, reason %d.",
                          p_ble_evt->evt.gap_evt.params.disconnected.reason);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
						Handle_disconnect();
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
    
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_DEBUG("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
            break;
        
        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
            break;

        case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
            break;

         case BLE_GAP_EVT_AUTH_STATUS:
             NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                          p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                          p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}
#if 1

#include "ble_link_ctx_manager.h"
/* Forward declaration of the ble_nus_t type. */
typedef struct ble_nus_s ble_nus_t;
/**@brief Nordic UART Service client context structure.
 *
 * @details This structure contains state context related to hosts.
 */
typedef struct
{
    bool is_notification_enabled; /**< Variable to indicate if the peer has enabled notification of the RX characteristic.*/
} ble_nus_client_context_t;

#define OPCODE_LENGTH        1
#define HANDLE_LENGTH        2

/**@brief   Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
    #define BLE_NUS_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)
#else
    #define BLE_NUS_MAX_DATA_LEN (BLE_GATT_MTU_SIZE_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH)
    #warning NRF_SDH_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

/**@brief   Macro for defining a ble_nus instance.
 *
 * @param     _name            Name of the instance.
 * @param[in] _nus_max_clients Maximum number of NUS clients connected at a time.
 * @hideinitializer
 */
#define BLE_NUS_DEF(_name, _nus_max_clients)                      \
    BLE_LINK_CTX_MANAGER_DEF(CONCAT_2(_name, _link_ctx_storage),  \
                             (_nus_max_clients),                  \
                             sizeof(ble_nus_client_context_t));   \
    static ble_nus_t _name =                                      \
    {                                                             \
        .p_link_ctx_storage = &CONCAT_2(_name, _link_ctx_storage) \
    };                                                            \
    NRF_SDH_BLE_OBSERVER(_name ## _obs,                           \
                         BLE_NUS_BLE_OBSERVER_PRIO,               \
                         ble_nus_on_ble_evt,                      \
                         &_name)

		
//BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */

uint32_t ble_nus_data_send(ble_nus_t * p_nus,
                           uint8_t   * p_data,
                           uint16_t  * p_length,
                           uint16_t    conn_handle)
{
    ret_code_t                 err_code;
    ble_gatts_hvx_params_t     hvx_params;
    ble_nus_client_context_t * p_client;

    VERIFY_PARAM_NOT_NULL(p_nus);

    //err_code = blcm_link_ctx_get(p_nus->p_link_ctx_storage, conn_handle, (void *) &p_client);
    VERIFY_SUCCESS(err_code);

    if ((conn_handle == BLE_CONN_HANDLE_INVALID) || (p_client == NULL))
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (!p_client->is_notification_enabled)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (*p_length > BLE_NUS_MAX_DATA_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    //hvx_params.handle = p_nus->tx_handles.value_handle;
    hvx_params.p_data = p_data;
    hvx_params.p_len  = p_length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(conn_handle, &hvx_params);
}
#endif

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */

void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;
	
		NRF_LOG_INFO("uart_event_handle===========================================");
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
					NRF_LOG_INFO("APP_UART_DATA_READY===========================================");
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r') ||
                (index >= m_ble_nus_max_data_len))
            {
                if (index > 1)
                {
                    NRF_LOG_INFO("Ready to send data over BLE NUS");
                    NRF_LOG_HEXDUMP_INFO(data_array, index);
										NRF_LOG_INFO("Ready to send data before do while over %d",index);
                    do
                    {
                        uint16_t length = (uint16_t)index;
												NRF_LOG_INFO("Ready to send data over %d",index);
//                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                            (err_code != NRF_ERROR_RESOURCES) &&
                            (err_code != NRF_ERROR_NOT_FOUND))
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                    } while (err_code == NRF_ERROR_RESOURCES);
                }

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
					NRF_LOG_INFO("APP_UART_COMMUNICATION_ERROR===========================================");
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
					NRF_LOG_INFO("APP_UART_FIFO_ERROR===========================================");
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_lesc_request_handler();
    APP_ERROR_CHECK(err_code);

    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

/**@brief Function for application main entry.
 */



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
static uint32_t m_data_receive[30];


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

uint32_t fstorage_data_write(uint32_t offset, void const * p_src, uint32_t len)
{
    ret_code_t rc = NRF_SUCCESS;
	  
    //memcpy(fstorage_data_write_temp + offset,p_src,len);
	  //return rc;
	
    VERIFY_PARAM_NOT_NULL(p_src);
    VERIFY_FALSE((len + offset > sizeof(fstorage_data_write_temp)), NRF_ERROR_INVALID_PARAM);

    //fstorage_data_lock();
		NRF_LOG_INFO("function:fstorage_data_write");
		//NRF_LOG_HEXDUMP_INFO(p_src, len);
	
    // prepare data
    rc = nrf_fstorage_read(&fstorage, fstorage.start_addr+offset,
                           fstorage_data_write_temp, sizeof(fstorage_data_write_temp));
    if (NRF_SUCCESS != rc)
    {
        NRF_LOG_INFO("nrf_fstorage_read fstorage init data Failed, 0x%x", rc);
        //goto WRITE_OUT;
    }
    
    memcpy(fstorage_data_write_temp + offset, p_src, len);
		//NRF_LOG_HEXDUMP_INFO(fstorage_data_write_temp + offset, len);

    // erase
    rc = nrf_fstorage_erase(&fstorage, fstorage.start_addr+offset, 1, NULL);
    if (NRF_SUCCESS != rc)
    {
        NRF_LOG_INFO("nrf_fstorage_erase Failed, rc=0x%x", rc);
        //goto WRITE_OUT;
    }
		
    //wait_for_flash_ready(&fstorage);
    
    // write
    rc = nrf_fstorage_write(&fstorage, fstorage.start_addr+offset,
                            fstorage_data_write_temp, sizeof(fstorage_data_write_temp), NULL);
    if (NRF_SUCCESS != rc)
    {
        NRF_LOG_INFO("nrf_fstorage_write Failed, offset=%d, len=%d, rc=0x%x", offset, len, rc);
        //goto WRITE_OUT;
    }

    //wait_for_flash_ready(&fstorage);


//WRITE_OUT:
    //fstorage_data_unlock();

    return rc;
}
uint32_t fstorage_data_write2(uint32_t offset, void const * p_src, uint32_t len)
{
    ret_code_t rc = NRF_SUCCESS;
	  
    //memcpy(fstorage_data_write_temp + offset,p_src,len);
	  //return rc;
	
    VERIFY_PARAM_NOT_NULL(p_src);
    VERIFY_FALSE((len + offset > sizeof(fstorage_data_write_temp)), NRF_ERROR_INVALID_PARAM);

    //fstorage_data_lock();
		NRF_LOG_INFO("function:fstorage_data_write");
		//NRF_LOG_HEXDUMP_INFO(p_src, len);
	#if 0
    // prepare data
    rc = nrf_fstorage_read(&fstorage, fstorage.start_addr,
                           fstorage_data_write_temp, sizeof(fstorage_data_write_temp));
    if (NRF_SUCCESS != rc)
    {
        NRF_LOG_INFO("nrf_fstorage_read fstorage init data Failed, 0x%x", rc);
        //goto WRITE_OUT;
    }
    
    memcpy(fstorage_data_write_temp + offset, p_src, len);
		//NRF_LOG_HEXDUMP_INFO(fstorage_data_write_temp + offset, len);
		//#if 0
    // erase
    rc = nrf_fstorage_erase(&fstorage, fstorage.start_addr, 1, NULL);
    if (NRF_SUCCESS != rc)
    {
        NRF_LOG_INFO("nrf_fstorage_erase Failed, rc=0x%x", rc);
        //goto WRITE_OUT;
    }
		#endif
    //wait_for_flash_ready(&fstorage);
    
    // write
    rc = nrf_fstorage_write(&fstorage, fstorage.start_addr+offset,
                            fstorage_data_write_temp, sizeof(fstorage_data_write_temp), NULL);
    if (NRF_SUCCESS != rc)
    {
        NRF_LOG_INFO("nrf_fstorage_write Failed, offset=%d, len=%d, rc=0x%x", offset, len, rc);
        //goto WRITE_OUT;
    }

    return rc;
}
static int storageReadData(uint8_t *outBuf, uint32_t word_count, uint32_t offset)
{
    uint32_t err_code;
		NRF_LOG_INFO("storageReadData");
    if (NULL == outBuf)
        return 0;

    if ((offset >= 1024) || (word_count + offset > 1024))
        return 0;
		
		if(word_count == 17){
		NRF_LOG_INFO("VIN-17");
			err_code = fstorage_data_read(offset, outBuf, word_count);
			//NRF_LOG_HEXDUMP_INFO(outBuf, word_count);
		}
		
		if(word_count == 48){
			NRF_LOG_INFO("CMPK-48");
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

int storageWriteData( unsigned char *inBuf, unsigned int word_count,unsigned int offset)
{
    uint32_t err_code;
    NRF_LOG_INFO("storageWriteData");
    if (NULL == inBuf)
        return 0;

    if ((offset >= 1024) || (word_count + offset > 1024))
        return 0;
		
		if(word_count == 48){
		NRF_LOG_INFO("CMPK-48");
			err_code = fstorage_data_write(offset, inBuf, word_count);
			//NRF_LOG_HEXDUMP_INFO(inBuf, word_count);
	}
		if(word_count == 17){
		NRF_LOG_INFO("VIN-17");
			err_code = fstorage_data_write(offset, inBuf, word_count);
			//NRF_LOG_HEXDUMP_INFO(inBuf, word_count);
	}
    if (NRF_SUCCESS != err_code)
    {
			NRF_LOG_INFO("error:storageWriteData function");
        return 0;
    }

    return 0;
}
#include "nrf_drv_clock.h"
static void clock_init(void)
{
    /* Initialize the clock. */
    ret_code_t rc = nrf_drv_clock_init();
    APP_ERROR_CHECK(rc);

    nrf_drv_clock_lfclk_request(NULL);

    // Wait for the clock to be ready.
    while (!nrf_clock_lf_is_running()) {;}
}
int main(void)
{
    bool erase_bonds;

		uart_init();
    log_init();
    timers_init();
	//#ifndef SOFTDEVICE_PRESENT
    clock_init();
	//#endif

    buttons_leds_init(&erase_bonds);
    power_management_init();

		fstorage_data_init();
		ble_stack_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
    //sensor_simulator_init();
    conn_params_init();
    peer_manager_init();

    // Start execution.
    NRF_LOG_INFO("started %s",DEVICE_NAME);
    application_timers_start();
    advertising_start(erase_bonds);
		
		#if 0
		uint8_t VIN_data1[40]={0x00,0x11,0x22,0x33,0x44,0x00,0x11,0x22,0x33,0x55,\
		0x00,0x11,0x22,0x33,0x44,0x00,0x11,0x22,0x33,0x44,\
			0x00,0x11,0x22,0x33,0x44,0x00,0x11,0x22,0x33,0x44,\
			0x00,0x11,0x22,0x33,0x44,0x00,0x11,0x22,0x33,0x44,\
			//0x00,0x11,0x22,0x33,0x44,0x00,0x11,0x22,0x33,0x44
		};
		uint8_t VIN_data2[60];
		

		storageWriteData(VIN_data1, sizeof(VIN_data1),0);
		NRF_LOG_INFO("main:VIN_data1");
		NRF_LOG_HEXDUMP_INFO(VIN_data1, sizeof(VIN_data1));

		
		storageReadData(VIN_data2,sizeof(VIN_data2),0);
		NRF_LOG_INFO("main:VIN_data2");
		NRF_LOG_HEXDUMP_INFO(VIN_data2, sizeof(VIN_data2));	
		
		
		NRF_LOG_INFO("test flash end");
		#endif //test flash 1
		
		#if 0
		uint8_t VIN_data1[10]={0x00,0x11,0x22,0x33,0x44,0x00,0x11,0x22,0x33,0x55};
		uint8_t VIN_data2[10];
		fstorage_data_read(0, VIN_data1, 5);
		NRF_LOG_INFO("main:VIN_data1:");
		NRF_LOG_HEXDUMP_INFO(VIN_data1, sizeof(VIN_data1));
		
    fstorage_data_write(0, VIN_data2, 5);
		NRF_LOG_INFO("main:VIN_data2:");
		NRF_LOG_HEXDUMP_INFO(VIN_data2, sizeof(VIN_data2));	
		#endif  //test flash 2
		
		#if 0 //test flash 3
		
		//nrf_fstorage_erase(&fstorage,fstorage.start_addr,1,NULL);
		
				uint8_t VIN_data1[56]={0x00,0x11,0x22,0x33,0x44,0x00,0x11,0x22,0x33,0x55,\
		   0x00,0x11,0x22,0x33,0x44,0x00,0x11,0x22,0x33,0x44,\
			0x00,0x11,0x22,0x33,0x44,0x00,0x11,0x22,0x33,0x44,\
			0x00,0x11,0x22,0x33,0x44,0x00,0x11,0x22,0x33,0x44,\
			0x00,0x11,0x22,0x33,0x44,0x00,0x11,0x22,0x33,0x50
		};
		//write flash

//		nrf_fstorage_write(&fstorage,fstorage.start_addr,VIN_data1,sizeof(VIN_data1),NULL);
//		NRF_LOG_HEXDUMP_INFO(VIN_data1, sizeof(VIN_data1));
		
		//read flash
	 nrf_fstorage_read(&fstorage, fstorage.start_addr,fstorage_data_write_temp, sizeof(fstorage_data_write_temp));
	 uint8_t VIN_data[FSTORAGE_DATA_SIZE_IN_BYTES];
	 memcpy(VIN_data,fstorage_data_write_temp,sizeof(fstorage_data_write_temp));
	 NRF_LOG_HEXDUMP_INFO(VIN_data, sizeof(VIN_data));	
		#endif //test flash 3
		
		#if 0
		//ingeek_set_callback(read_CB1,write_CB1,Rand_CB1);
		ingeek_set_callback(storageReadData,storageWriteData,Rand_CB1);
		#else
		ingeek_set_callback(storageReadData,storageWriteData,ikif_random_vector_generate);
		//ingeek_set_callback(read_CB1,storageWriteData,ikif_random_vector_generate);
		#endif
		
		//NRF_LOG_INFO("after:ingeek_set_callback");
		ingeek_se_init();
    // Enter main loop.
    for (;;)
    {	
        idle_state_handle();
    }
}


