#ifndef _BLE_INGEEK_H
#define _BLE_INGEEK_H

#include <stdint.h>
#include "ble.h"
#include <stdbool.h>//add lifei
#include "ble_link_ctx_manager.h" //add lifei
/**@brief   Macro for defining a ble_ingeek instance.
 *
 * @param     _name            Name of the instance.
 * @param[in] _nus_max_clients Maximum number of NUS clients connected at a time.
 * @hideinitializer
 */
#define BLE_INGEEK_DEF(_name, _ingeek_max_clients)                      \
    BLE_LINK_CTX_MANAGER_DEF(CONCAT_2(_name, _link_ctx_storage),  \
                             (_ingeek_max_clients),                  \
                             sizeof(ble_ingeek_client_context_t));   \
    static ble_ingeek_t _name =                                      \
    {                                                             \
        .p_link_ctx_storage = &CONCAT_2(_name, _link_ctx_storage) \
    };                                                            \
    NRF_SDH_BLE_OBSERVER(_name ## _obs,                           \
                         BLE_INGEEK_BLE_OBSERVER_PRIO,               \
                         ble_ingeek_on_ble_evt,                      \
                         &_name)

		
#define INGEEK_UUID_BASE        {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, \
                              0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}

#define INGEEK_UUID_SERVICE     0x1500
#define INGEEK_UUID_BUTTON_CHAR 0x1501
#define INGEEK_UUID_LED_CHAR    0x1502
															
/**@brief Nordic UART Service client context structure.
 *
 * @details This structure contains state context related to hosts.
 */
typedef struct
{
    bool is_notification_enabled; /**< Variable to indicate if the peer has enabled notification of the RX characteristic.*/
} ble_ingeek_client_context_t;//add lifei
															
// Forward declaration of the ble_lbs_t type. 
typedef struct ble_ingeek_s ble_ingeek_t;															
															
typedef void (*ble_ingeek_led_write_handler_t) (ble_ingeek_t * p_lbs, uint8_t new_state);															
															
/** @brief LED Button Service init structure. This structure contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_ingeek_led_write_handler_t led_write_handler; /**< Event handler to be called when the LED Characteristic is written. */
} ble_ingeek_init_t;
															

/**@brief LED Button Service structure. This structure contains various status information for the service. */
struct ble_ingeek_s
{
    uint16_t                    ingeek_service_handle;      /**< Handle of LED Button Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    ingeek_led_char_handles;    /**< Handles related to the LED Characteristic. */
    ble_gatts_char_handles_t    ingeek_button_char_handles; /**< Handles related to the Button Characteristic. */
    uint8_t                     ingeek_uuid_type;           /**< UUID type for the LED Button Service. */
    uint16_t                    ingeek_conn_handle;         /**< Handle of the current connection (as provided by the BLE stack). BLE_CONN_HANDLE_INVALID if not in a connection. */
	  blcm_link_ctx_storage_t * const p_link_ctx_storage;  /*add lifei*/
    ble_ingeek_led_write_handler_t ingeek_led_write_handler;   /**< Event handler to be called when the LED Characteristic is written. */
};


/**@brief Function for initializing the LED Button Service.
 *
 * @param[out] p_lbs      LED Button Service structure. This structure must be supplied by
 *                        the application. It is initialized by this function and will later
 *                        be used to identify this particular service instance.
 * @param[in] p_lbs_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was initialized successfully. Otherwise, an error code is returned.
 */
uint32_t ble_ingeek_init(ble_ingeek_t * p_lbs, const ble_ingeek_init_t * p_lbs_init);


/**@brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to the LED Button Service.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
//void ble_ingeek_on_ble_evt(ble_ingeek_t * p_lbs, ble_evt_t * p_ble_evt);
void ble_ingeek_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);
/**@brief Function for sending a button state notification.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] button_state  New button state.
 *
 * @retval NRF_SUCCESS If the notification was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_ingeek_on_button_change(ble_ingeek_t * p_lbs, uint8_t button_state);


#endif
