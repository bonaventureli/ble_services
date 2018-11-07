#include "ble_ingeek.h"
#include "ble_srv_common.h"
#include "sdk_common.h"
#include "ble.h"

/**@brief Function for handling the Connect event.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
 static void on_connect(ble_ingeek_t * p_lbs, ble_evt_t const * p_ble_evt)//lifei
//static void on_connect(ble_ingeek_t * p_lbs, ble_evt_t * p_ble_evt)
{
    p_lbs->ingeek_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}
/**@brief Function for handling the Disconnect event.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_disconnect(ble_ingeek_t * p_lbs, ble_evt_t const* p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_lbs->ingeek_conn_handle = BLE_CONN_HANDLE_INVALID;
}
/**@brief Function for handling the Write event.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
//static void on_write(ble_ingeek_t * p_lbs, ble_evt_t * p_ble_evt)
	static void on_write(ble_ingeek_t * p_lbs, ble_evt_t const * p_ble_evt)//lifei
{
    ble_gatts_evt_write_t const* p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if ((p_evt_write->handle == p_lbs->ingeek_led_char_handles.value_handle) &&
        (p_evt_write->len == 1) &&
        (p_lbs->ingeek_led_write_handler != NULL))//判断是不是读操作
    {
        p_lbs->ingeek_led_write_handler(p_lbs, p_evt_write->data[0]);//写数据
    }
}
#if 0
void ble_ingeek_on_ble_evt(ble_ingeek_t * p_lbs, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_lbs, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_lbs, p_ble_evt);
            break;
            
        case BLE_GATTS_EVT_WRITE:
            on_write(p_lbs, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}
#endif

void ble_ingeek_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
	//void ble_nus_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    ble_ingeek_t * p_ingeek = (ble_ingeek_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_ingeek, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_ingeek, p_ble_evt);
            break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
           // on_hvx_tx_complete(p_ingeek, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for adding the LED Characteristic.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_lbs_init LED Button Service initialization structure.
 *
 * @retval NRF_SUCCESS on success, else an error value from the SoftDevice
 */
static uint32_t led_char_add(ble_ingeek_t * p_lbs, const ble_ingeek_init_t * p_lbs_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));
  //特征描述符

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_lbs->ingeek_uuid_type;
    ble_uuid.uuid = INGEEK_UUID_LED_CHAR;
    
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);//不需要加密
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;//读写不需要认证
    attr_md.vlen       = 0;//不可变长度属性 
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));
//特征值的属性
    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;//安全属性
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint8_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_lbs->ingeek_service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_lbs->ingeek_led_char_handles);
}

/**@brief Function for adding the Button Characteristic.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_lbs_init LED Button Service initialization structure.
 *
 * @retval NRF_SUCCESS on success, else an error value from the SoftDevice
 */
static uint32_t button_char_add(ble_ingeek_t * p_lbs, const ble_ingeek_init_t * p_lbs_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
		char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_lbs->ingeek_uuid_type;
    ble_uuid.uuid = INGEEK_UUID_BUTTON_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint8_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_lbs->ingeek_service_handle,
                                               &char_md,
                                               &attr_char_value,
                                               &p_lbs->ingeek_button_char_handles);
}
//uint32_t ble_ingeek_init_new(ble_nus_t * p_nus, ble_nus_init_t const * p_nus_init)
uint32_t ble_ingeek_init(ble_ingeek_t * p_lbs, const ble_ingeek_init_t * p_lbs_init)
{
    ret_code_t            err_code;
    ble_uuid_t            ble_uuid;
    ble_uuid128_t         nus_base_uuid = INGEEK_UUID_BASE;
    ble_add_char_params_t add_char_params;

    VERIFY_PARAM_NOT_NULL(p_lbs);
    VERIFY_PARAM_NOT_NULL(p_lbs_init);

    // Initialize the service structure.
//    p_nus->data_handler = p_nus_init->data_handler;
	  p_lbs->ingeek_led_write_handler = p_lbs_init->led_write_handler;

    /**@snippet [Adding proprietary Service to the SoftDevice] */
    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&nus_base_uuid, &p_lbs->ingeek_uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_lbs->ingeek_uuid_type;
    ble_uuid.uuid = INGEEK_UUID_SERVICE;

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_lbs->ingeek_service_handle);
    /**@snippet [Adding proprietary Service to the SoftDevice] */
    VERIFY_SUCCESS(err_code);

    // Add the RX Characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                     = INGEEK_UUID_BUTTON_CHAR;
    add_char_params.uuid_type                = p_lbs->ingeek_uuid_type;
    //add_char_params.max_len                  = BLE_NUS_MAX_RX_CHAR_LEN;
    add_char_params.init_len                 = sizeof(uint8_t);
    add_char_params.is_var_len               = true;
    add_char_params.char_props.write         = 1;
    add_char_params.char_props.write_wo_resp = 1;

    add_char_params.read_access  = SEC_OPEN;
    add_char_params.write_access = SEC_OPEN;

    err_code = characteristic_add(p_lbs->ingeek_service_handle, &add_char_params, &p_lbs->ingeek_led_char_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the TX Characteristic.
    /**@snippet [Adding proprietary characteristic to the SoftDevice] */
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = INGEEK_UUID_LED_CHAR;
    add_char_params.uuid_type         = p_lbs->ingeek_uuid_type;
    //add_char_params.max_len           = BLE_NUS_MAX_TX_CHAR_LEN;
    add_char_params.init_len          = sizeof(uint8_t);
    add_char_params.is_var_len        = true;
    add_char_params.char_props.notify = 1;

    add_char_params.read_access       = SEC_OPEN;
    add_char_params.write_access      = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;

    return characteristic_add(p_lbs->ingeek_service_handle, &add_char_params, &p_lbs->ingeek_button_char_handles);
    /**@snippet [Adding proprietary characteristic to the SoftDevice] */
}
#if 0
uint32_t ble_ingeek_init(ble_ingeek_t * p_lbs, const ble_ingeek_init_t * p_lbs_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure.初始化服务结构
    p_lbs->ingeek_conn_handle       = BLE_CONN_HANDLE_INVALID;// 初始化连接句柄，因为现在并未与手机连接所以先赋值无效。
    p_lbs->ingeek_led_write_handler = p_lbs_init->led_write_handler; //赋值数据处理函数，就是控制硬件的LED处理

    // Add service.
    ble_uuid128_t base_uuid = {INGEEK_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_lbs->ingeek_uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_lbs->ingeek_uuid_type;
    ble_uuid.uuid = INGEEK_UUID_SERVICE;
   //GATT服务添加，主要是添加服务UUID
		err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_lbs->ingeek_service_handle);
		
    VERIFY_SUCCESS(err_code);

    // Add characteristics.添加特征值
    err_code = button_char_add(p_lbs, p_lbs_init);
    VERIFY_SUCCESS(err_code);

    err_code = led_char_add(p_lbs, p_lbs_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}
#endif

uint32_t ble_ingeek_on_button_change(ble_ingeek_t * p_lbs, uint8_t button_state)
{
    ble_gatts_hvx_params_t params;
    uint16_t len = sizeof(button_state);
    
    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_lbs->ingeek_button_char_handles.value_handle;
    params.p_data = &button_state;
    params.p_len = &len;
    
    return sd_ble_gatts_hvx(p_lbs->ingeek_conn_handle, &params);
}
