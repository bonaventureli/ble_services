#ifndef _BLE_SDK_H
#define _BLE_SDK_H

#define SDKUART 1
#define HRS 0

#include "ble.h"
#include "ble_gatt.h"


#if SDKUART
#include "app_uart.h"
#include "ble_link_ctx_manager.h"

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#endif


#include "nrf_log.h"

#define BLE_SDK_BLE_OBSERVER_PRIO 2
//typedef void (*nrf_sdh_ble_evt_handler_t)(ble_evt_t const * p_ble_evt, void * p_context);
//typedef void (*ble_sdk_info_handler_t) (ble_sdk_t * p_lbs, uint8_t * infoValue, uint16_t InfoLen);	


#define BLE_SDK_DEF(_name)                                                                          \
static ble_sdk_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_SDK_BLE_OBSERVER_PRIO,                                                     \
                     ble_sdk_on_ble_evt, &_name)



//#define BLE_SDKSERVER_DEF(_name, _sdk_max_clients)                      \
//    BLE_LINK_CTX_MANAGER_DEF(CONCAT_2(_name, _link_ctx_storage),  \
//                             (_sdk_max_clients),                  \
//                             sizeof(ble_sdk_client_context_t));   \
//    static ble_sdk_t _name =                                      \
//    {                                                             \
//        .p_link_ctx_storage = &CONCAT_2(_name, _link_ctx_storage) \
//    };                                                            \
//    NRF_SDH_BLE_OBSERVER(_name ## _obs,                           \
//                         BLE_SDK_BLE_OBSERVER_PRIO,               \
//                         ble_sdk_on_ble_evt,                      \
//                         &_name)
		
		
#define SDK_UUID_BASE        {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, \
                              0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}

#define SDK_UUID_SERVICE     0xFFF0
#define INFO_UUID_CHAR 		0xFFF1
#define AUTH_UUID_CHAR    0xFFF2
#define SESSION_UUID_CHAR    0xFFF3
#define CMD_UUID_CHAR    0xFFF4
#define RSSI_UUID_CHAR    0xFFF5

															
#define SDK_INFO_LEN					128					//info len
#define SDK_AUTH_LEN					128					//auth len
#define SDK_SESSION_LEN				128					//session len
#define SDK_CMD_LEN						20					//cmd len
#define SDK_RSSI_LEN					5					//rssi len			
//#define BLE_SDK_MAX_DATA_LEN	(GATT_MTU_SIZE_DEFAULT - 3)					//	(GATT_MTU_SIZE_DEFAULT - 3)
#define BLE_SDK_MAX_DATA_LEN	(BLE_GATT_ATT_MTU_DEFAULT - 3)					//	(GATT_MTU_SIZE_DEFAULT - 3)
															
// Forward declaration of the ble_lbs_t type. 
typedef struct ble_sdk_s ble_sdk_t;		

/**@brief   Nordic SDK Service event types. */
typedef enum
{
    BLE_SDK_EVT_RX_DATA,      /**< Data received. */
    BLE_SDK_EVT_TX_RDY,       /**< Service is ready to accept new data to be transmitted. */
    BLE_SDK_EVT_COMM_STARTED, /**< Notification has been enabled. */
    BLE_SDK_EVT_COMM_STOPPED, /**< Notification has been disabled. */
} ble_sdk_evt_type_t;
															
/**@brief Nordic SDK Service client context structure.
 *
 * @details This structure contains state context related to hosts.
 */
typedef struct
{
    bool is_notification_enabled; /**< Variable to indicate if the peer has enabled notification of the RX characteristic.*/
} ble_sdk_client_context_t;
/**@brief   Nordic SDK Service @ref BLE_NUS_EVT_RX_DATA event data.
 *
 * @details This structure is passed to an event when @ref BLE_NUS_EVT_RX_DATA occurs.
 */
typedef struct
{
    uint8_t const * p_data; /**< A pointer to the buffer with received data. */
    uint16_t        length; /**< Length of received data. */
} ble_sdk_evt_rx_data_t;

/**@brief   Nordic SDK Service event structure.
 *
 * @details This structure is passed to an event coming from service.
 */
typedef struct
{
    ble_sdk_evt_type_t         type;        /**< Event type. */
    ble_sdk_t                * p_sdk;       /**< A pointer to the instance. */
    uint16_t                   conn_handle; /**< Connection handle. */
    ble_sdk_client_context_t * p_link_ctx;  /**< A pointer to the link context. */
    union
    {
        ble_sdk_evt_rx_data_t rx_data; /**< @ref BLE_NUS_EVT_RX_DATA event data. */
    } params;
} ble_sdk_evt_t;
															
typedef void (*ble_sdk_data_handler_t) (ble_sdk_evt_t * p_evt);														
typedef void (*ble_sdk_info_handler_t) (ble_sdk_t * p_lbs, uint8_t * infoValue, uint16_t InfoLen);																
typedef void (*ble_sdk_auth_handler_t) (ble_sdk_t * p_lbs, uint8_t * authValue, uint16_t AuthLen);																
typedef void (*ble_sdk_session_handler_t) (ble_sdk_t * p_lbs, uint8_t * sessionValue, uint16_t SessionLen);																
typedef void (*ble_sdk_cmd_handler_t) (ble_sdk_t * p_lbs, uint8_t * cmdValue, uint16_t CmdLen);																
typedef void (*ble_sdk_rssi_handler_t) (ble_sdk_t * p_lbs, uint8_t * rssiValue, uint16_t RssiLen);																															

/** @brief LED Button Service init structure. This structure contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_sdk_data_handler_t			data_handler;
	  ble_sdk_info_handler_t 			info_write_function_handler;
	  ble_sdk_auth_handler_t 			auth_write_function_handler;
	  ble_sdk_session_handler_t 	session_write_function_handler;
	  ble_sdk_cmd_handler_t 			cmd_write_function_handler;
	  ble_sdk_rssi_handler_t 			rssi_write_function_handler;
} ble_sdk_init_t;
															
/**@brief LED Button Service structure. This structure contains various status information for the service. */
struct ble_sdk_s
{
	
    uint16_t                      sdk_service_handle;      /**< Handle of LED Button Service (as provided by the BLE stack). */
	  uint8_t                     sdk_uuid_type;           /**< UUID type for the LED Button Service. */
    uint16_t                    sdk_conn_handle;         /**< Handle of the current connection (as provided by the BLE stack). BLE_CONN_HANDLE_INVALID if not in a connection. */
	
		ble_gatts_char_handles_t 			sdk_info_char_handler;
		ble_gatts_char_handles_t 			sdk_auth_char_handler;
		ble_gatts_char_handles_t 			sdk_session_char_handler;
		ble_gatts_char_handles_t 			sdk_cmd_char_handler;
		ble_gatts_char_handles_t 			sdk_rssi_char_handler;
	  ble_sdk_data_handler_t			data_handler;
	
    
	  ble_sdk_info_handler_t 			sdk_info_handler;
	  ble_sdk_auth_handler_t 			sdk_auth_handler;
	  ble_sdk_session_handler_t 	sdk_session_handler;
	  ble_sdk_cmd_handler_t 			sdk_cmd_handler;
	  ble_sdk_rssi_handler_t 			sdk_rssi_handler;
	
	#if SDKUART
		blcm_link_ctx_storage_t * const p_link_ctx_storage; /**< Pointer to link context storage with handles of all current connections and its context. */
  #endif
};
typedef struct
{
		uint8_t *data;
		uint16_t len;
		uint16_t offset;
} data_info;

/**@brief Function for initializing the LED Button Service.
 *
 * @param[out] p_lbs      LED Button Service structure. This structure must be supplied by
 *                        the application. It is initialized by this function and will later
 *                        be used to identify this particular service instance.
 * @param[in] p_lbs_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was initialized successfully. Otherwise, an error code is returned.
 */
uint32_t ble_sdk_init(ble_sdk_t * p_lbs, const ble_sdk_init_t * p_lbs_init);

/**@brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to the LED Button Service.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
//void ble_sdk_on_ble_evt(ble_sdk_t * p_lbs, ble_evt_t * p_ble_evt);
void ble_sdk_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

uint32_t ble_SDK_char_set(ble_sdk_t * p_lbs, uint16_t uuid, uint8_t * showValues, uint16_t valueLen);

#endif
