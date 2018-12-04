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
//#include "fstorage_data.h"

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


uint8_t VIN_data[17];
uint8_t CMPK_data[48];

uint8_t VIN_CMPK_data[70];//65

uint8_t Rand_data[32]={
0xB1,0xB1,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBF,
0xB1,0xB1,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBF,
0xB1,0xB1,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBF,
0xB1,0xB1
};

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
		memcpy(out,VIN_data,rlen);
	}
	if(rlen == 48){
		NRF_LOG_INFO("CMPK");
		memcpy(out,CMPK_data,rlen);
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

	if(wlen == 48){
	NRF_LOG_INFO("CMPK-48");
	memcpy(CMPK_data,in,wlen);
	}
		if(wlen == 17){
	NRF_LOG_INFO("VIN-17");
	memcpy(VIN_data,in,wlen);
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
		//Receive_Task();
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
	
	// One,push info
	ingeek_push_info(data, data_len);

	//Two,Notify status
	status = ingeek_get_sec_status();
	ble_send_notify(BLE_UUID_DIGITALKET_STATUS_CHAR, &status, 1);
	
	//Three,pull info
	ingeek_pull_info(gReturnInfo, &gReturnInfoLen);
	NRF_LOG_INFO("Handle_info:[After] ingeek_pull_info:status:ingeek_get_sec_status is %x. if 1,it is right\n",ingeek_get_sec_status());
	//Fore,Notify info
	ble_send_notify(BLE_UUID_DIGITALKET_INFO_CHAR, gReturnInfo, gReturnInfoLen);
	//ikble_set_se_info(gReturnInfo,gReturnInfoLen);

}
void Handle_auth(uint8_t *data, uint32_t data_len)
{
	uint8_t status;
	
	#if 1
	status = ingeek_get_sec_status();
	
	NRF_LOG_INFO("\nHandle_auth_function:[Before] ingeek_push_auth:status:ingeek_get_sec_status is %x. if 1,it is right\n",ingeek_get_sec_status());
	NRF_LOG_INFO("\nHandle_auth_function,return:ingeek_push_auth %x, if 0,it is right\n",ingeek_push_auth(data, data_len, (unsigned char*)1, (unsigned int*)1));
	NRF_LOG_INFO("\nHandle_auth_function:[After] ingeek_push_auth:status:ingeek_get_sec_status is %x. if 2,it is right\n",ingeek_get_sec_status());
	status = ingeek_get_sec_status();
	if(status == 0x02)
	ble_send_notify(BLE_UUID_DIGITALKET_STATUS_CHAR, &status, 1);
	#endif
	
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
	//T_UartSendSession FramedataSession;
	unsigned int outlen;
	uint8_t *preply_data;
	uint8_t status;
	//preply_data = gDataload;
	#if 0
	NRF_LOG_INFO("\nHandle_session_function:[Before] ingeek_push_session:status:ingeek_get_sec_status is %x. if 2,it is right\n",ingeek_get_sec_status());
	NRF_LOG_INFO("\nHandle_session_function,return:ingeek_push_session %x, if 0,it is right\n",ingeek_push_session(data, data_len, preply_data, &outlen));
	NRF_LOG_INFO("\nHandle_session_function:[After] ingeek_push_session:status:ingeek_get_sec_status is %x. if 3,it is right\n",ingeek_get_sec_status());
	
	NRF_LOG_INFO("\nHandle_session is ok ,ingeek_push_session,preply_data:\n");
	#endif

	ingeek_push_session(data, data_len, gReturnSession, &gReturnSessionLen);
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
//	T_UartSendCmd FramedataCmd;
	uint8_t *preply_data;
	unsigned int outlen;
	

	DK_Cmd_Meg struct_cmd;
	uint8_t cmd;

//	preply_data = gDataload;
	#if 1
	NRF_LOG_HEXDUMP_INFO(data, data_len);
	NRF_LOG_INFO("\nHandle_cmd_function:[Before] ingeek_command_input_action:status:ingeek_get_sec_status is %x. if 3,it is right\n",ingeek_get_sec_status());
	NRF_LOG_INFO("\nHandle_cmd_function,return:ingeek_command_input_action %x, if 0,it is right\n",ingeek_command_input_action(data, data_len, &struct_cmd));
	NRF_LOG_INFO("\nHandle_cmd_function:[After] ingeek_command_input_action:status:ingeek_get_sec_status is %x. if 3,it is right\n",ingeek_get_sec_status());
	NRF_LOG_INFO("\nHandle_cmd_command: %d \n",(uint8_t)(struct_cmd.command));
	NRF_LOG_INFO("\nHandle_cmd_index %d \n",(uint8_t)(struct_cmd.index));
	NRF_LOG_INFO("\nHandle_cmd_result %d \n",(uint8_t)(struct_cmd.result));
	NRF_LOG_INFO("\nHandle_cmd_permission %d \n",(uint8_t)(struct_cmd.permission));
	NRF_LOG_INFO("\nHandle_cmd_sparam_size %d \n",(uint8_t)(struct_cmd.sparam_size));
	NRF_LOG_HEXDUMP_INFO((uint8_t *)struct_cmd.sparam, struct_cmd.sparam_size);
	NRF_LOG_INFO("\n=========================================");
	ikcmdSendUart(struct_cmd.command);
	#if 1
	if(struct_cmd.command == 1){
		
	struct_cmd.result = 1;
		#if 0
	struct_cmd.index = 1;
	struct_cmd.permission = 1;
	struct_cmd.sparam_size = 5;
	memcpy(struct_cmd.sparam, (uint8_t*)&CarCMD, sizeof(T_CarCMD));
	NRF_LOG_HEXDUMP_INFO((uint8_t *)&struct_cmd, sizeof(struct_cmd));
		#endif
		NRF_LOG_INFO("\nHandle_cmd_function 1111");
	}
	else if(struct_cmd.command == 2){
	struct_cmd.result = 2;
		#if 0
	struct_cmd.index = 2;
	struct_cmd.permission = 2;
	struct_cmd.sparam_size = 5;
	memcpy(struct_cmd.sparam, (uint8_t*)&CarCMD, sizeof(T_CarCMD));
	NRF_LOG_HEXDUMP_INFO((uint8_t *)&struct_cmd, sizeof(struct_cmd));
		#endif
		NRF_LOG_INFO("\nHandle_cmd_function 2222");
	}
	#endif
	#endif
	
	#if 0
	ingeek_command_input_action(data, data_len, &struct_cmd);
	#endif
	
	#if 1
	NRF_LOG_INFO("\nHandle_cmd_function,return:ingeek_command_output_action %x, if 0,it is right\n",ingeek_command_output_action(&struct_cmd,preply_data, &outlen));
	NRF_LOG_INFO("\nHandle_cmd is ok ,ingeek_command_input_action,preply_data:\n");
	
	#endif
	
	#if 0
	ingeek_command_output_action(&struct_cmd,preply_data, &outlen);
	//NRF_LOG_HEXDUMP_INFO(preply_data, outlen);
	NRF_LOG_HEXDUMP_INFO(preply_data, outlen);
	#endif
	
	ble_send_notify(BLE_UUID_DIGITALKET_CMD_CHAR, preply_data, outlen);

}
#define SESSIONMAXBIT 112
#define INFOCHARMAXBIT 29
#define DK_REPLAY_MAXBIT 128
void ble_hrs_evt_handler (ble_hrs_t * p_hrs, ble_hrs_evt_t * p_evt){

	uint8_t status = 0;
	NRF_LOG_INFO("ble_hrs_evt_handler function==========");
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
			NRF_LOG_HEXDUMP_INFO((uint8_t *)p_evt->params.rx_data.p_data, (uint16_t)p_evt->params.rx_data.length);	
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
		return;
	}
	#if 0
	if((ret >= 0)&&(ret < 0xFF)){
		NRF_LOG_INFO("status %x",ret);
		Handle_active();
	}
	#endif
	if((ret == 0) || (ret == 0x01)){
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
#if 0
/*!
 * typedef int (*pfnReadPortCB_t)(eDKPORT port,uint8_t *outBuf, uint32_t size,uint32_t offset)
 * read content from flash/uart
 * @param [out] outBuf: read flash or uart content and write to outBuf
 * @param [in] size: outBuf size .
 * @param [in] offset:  read flash offset position
 * @return 	actual read size from port.
 * Example usage:
 * @verbatim
 * @endverbatim
 */
typedef int (*pfnReadPortCB_t)(uint8_t *outBuf, uint32_t size,uint32_t offset);
/*!
 * typedef int (*pfnWritePortCB_t)(const uint8_t *inBuf, uint32_t size,uint32_t offset)
 * write content to flash or uart
 * @param [in] inBuf: content will be writed to flash or uart.
 * @param [in] size: inBuf size .
 * @param [in] offset:  write to flash offset position
 * @return actual write size to port.
 * Example usage:
 * @verbatim
 * @endverbatim
 */
typedef int (*pfnWritePortCB_t)(const uint8_t *inBuf, uint32_t size,uint32_t offset);
/*!
 *  \struct dkIOs_t
 *  \brief 
 */
typedef struct
{
   pfnWritePortCB_t     uartTxCallback; //write uart callback funtion point
   pfnReadPortCB_t      flashRCallback; //read flash callback funtion point
   pfnWritePortCB_t     flashWCallback; //write flash callback funtion point
} dkIOs_t;
#define SESSION_MAX 1
dkIOs_t IKTK_SessionContain[SESSION_MAX] ={0};
pfnWritePortCB_t   g_uartTxCB = 0;
int ikcmdInitialize(pfnWritePortCB_t inUartTx)
{
    g_uartTxCB = inUartTx;
	  return 0;
}
uint32_t DKCreateSRV(dkIOs_t * pSession)
{
    static uint32_t session_id = 0;
  if (pSession && session_id < SESSION_MAX) {
      ++session_id;
      //ikbleInitialize();
      IKTK_SessionContain[session_id - 1] = *pSession;
      ikcmdInitialize(pSession->uartTxCallback);
      //ingeek_set_callback(pSession->flashRCallback,pSession->flashWCallback,ikif_random_vector_generate);
	  //ikSecuritySetRunningStatus(DK_SYS_INIT);
      return session_id;
  } else {
    return 0;
  }
}
static dkIOs_t g_ios = {0,0,0};

static int storageReadData(uint8_t *outBuf, uint32_t word_count, uint32_t offset)
{
    uint32_t err_code;
    if (NULL == outBuf)
        return 0;

    if ((offset >= 1024) || (word_count + offset > 1024))
        return 0;
        
    err_code = fstorage_data_read(offset, outBuf, word_count);
    if (NRF_SUCCESS != err_code)
    {
        return 0;
    }

    return 0;
}

static int storageWriteData(const uint8_t *inBuf, uint32_t word_count,uint32_t offset)
{
    uint32_t err_code;
    
    if (NULL == inBuf)
        return 0;

    if ((offset >= 1024) || (word_count + offset > 1024))
        return 0;
        
    err_code = fstorage_data_write(offset, inBuf, word_count);
    if (NRF_SUCCESS != err_code)
    {
        return 0;
    }

    return 0;
}
#endif

#if 0
static int ikif_uart_txcb( const uint8_t *msg, uint32_t wlen, uint32_t offset )
{
    int ret;
    //NRF_LOG_INFO("ikif_uart_txcb tx %d bytes", wlen);
    
    ret = m2m_if_UartSendData((uint8_t *)msg, (uint16_t)wlen);
    if (wlen != ret)
    {
        NRF_LOG_WARNING("m2m_if_UartSendData send Failed, req=%d, sended=%d", wlen, ret);
    }
    
    return ret;
}
#endif
/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;

    // Initialize.
		
	
    log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
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
		
//		// Add fstorage data
//    (void)fstorage_data_init();
//		// Add ingeek function
//    g_ios.flashRCallback = storageReadData;
//    g_ios.flashWCallback = storageWriteData;
//    //g_ios.uartTxCallback = ikif_uart_txcb;
//    DKCreateSRV(&g_ios);
		//NRF_LOG_INFO("\n ========================Start:ingeek_set_callback function=====================\n \n");
		ingeek_set_callback(read_CB1,write_CB1,Rand_CB1);
		//NRF_LOG_INFO("\n ========================End:ingeek_set_callback function=======================\n \n");
		//ingeek_set_callback(read_CB1,write_CB1,ikif_random_vector_generate);
		
		
		//NRF_LOG_INFO("main:return-ingeek_se_init is %x.",ingeek_se_init());
		//NRF_LOG_INFO("\n ========================Start:ingeek_se_init function=====================\n \n");
		ingeek_se_init();
		//NRF_LOG_INFO("\n ========================End:ingeek_se_init function=======================\n \n");
		
		NRF_LOG_INFO("main:return-ingeek_get_sec_status is %x.",ingeek_get_sec_status());
#include "cipher_export.h"
unsigned char vkey[16]={0xf1,0xf1,0xf1,0xf1,0xf1,0xf1,0xf1,0xf1,0xf1,0xf1,0xf1,0xf1,0xf1,0xf1,0xf1,0x54};//key
unsigned char viv[16]={0x78,0x78,0x78,0x78,0x78,0x78,0x78,0x78,0x78,0x78,0x78,0x78,0x78,0x78,0x78,0x78};//vector 
unsigned char in[16]={0x12,0x12,0x12,0x12,0x12,0x12,0x12,0x12,0x12,0x12,0x12,0x12,0x12,0x12,0x12,0x12};
	unsigned int ilen = 16;
	unsigned char out[32];
	unsigned int olen;

	unsigned char out2[48]={0};
	unsigned int olen2;
	NRF_LOG_HEXDUMP_INFO(in, ilen);
	ingeek_cipher_aes_cbc(vkey,viv,in,ilen,out,&olen,INGEEK_ENCRYPT);
	NRF_LOG_HEXDUMP_INFO(out, olen);
	NRF_LOG_INFO("olen %d.",olen);
	
	#if 1
	ingeek_cipher_aes_cbc(vkey,viv,out,olen,out2,&olen2,INGEEK_DECRYPT);
	NRF_LOG_HEXDUMP_INFO(out2, olen2);
	NRF_LOG_INFO("olen2 %d.",olen2);
	#endif
	
	
	
	
    // Enter main loop.
    for (;;)
    {
			#if 0
			int ingeek_cipher_aes_cbc(unsigned char* key,unsigned char*iv,
							unsigned char*in,unsigned int ilen,
							unsigned char*out,unsigned int *olen,
							ingeek_operation_t mode);
			#endif
				  
							
        idle_state_handle();
    }
}


