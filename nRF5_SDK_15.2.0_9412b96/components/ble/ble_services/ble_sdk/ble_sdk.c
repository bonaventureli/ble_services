#include "ble_sdk.h"
#include "ble_srv_common.h"
#include "sdk_common.h"

//static data_info g_send_data = {NULL, 0, 0};
static data_info g_rcv_data = {NULL, 0, 0};

/**@brief Function for handling the Connect event.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_connect(ble_sdk_t * p_sdk, ble_evt_t const * p_ble_evt)
{
    //p_lbs->sdk_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
	#if SDKUART
		ret_code_t                 err_code;
    ble_sdk_evt_t              evt;
    ble_gatts_value_t          gatts_val;
    uint8_t                    cccd_value[2];
    ble_sdk_client_context_t * p_client = NULL;

    err_code = blcm_link_ctx_get(p_sdk->p_link_ctx_storage,
                                 p_ble_evt->evt.gap_evt.conn_handle,
                                 (void *) &p_client);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
                      p_ble_evt->evt.gap_evt.conn_handle);
    }

    /* Check the hosts CCCD value to inform of readiness to send data using the RX characteristic */
    memset(&gatts_val, 0, sizeof(ble_gatts_value_t));
    gatts_val.p_value = cccd_value;
    gatts_val.len     = sizeof(cccd_value);
    gatts_val.offset  = 0;

    err_code = sd_ble_gatts_value_get(p_ble_evt->evt.gap_evt.conn_handle,
                                      p_sdk->sdk_info_char_handler.cccd_handle,
                                      &gatts_val);

    if ((err_code == NRF_SUCCESS)     &&
        (p_sdk->data_handler != NULL) &&
        ble_srv_is_notification_enabled(gatts_val.p_value))
    {
        if (p_client != NULL)
        {
            p_client->is_notification_enabled = true;
        }

        memset(&evt, 0, sizeof(ble_sdk_evt_t));
        evt.type        = BLE_SDK_EVT_COMM_STARTED;
        evt.p_sdk       = p_sdk;
        evt.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        evt.p_link_ctx  = p_client;

        p_sdk->data_handler(&evt);
    }
		#endif
}
/**@brief Function for handling the Disconnect event.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_disconnect(ble_sdk_t * p_lbs, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_lbs->sdk_conn_handle = BLE_CONN_HANDLE_INVALID;
}
void SDK_data_free_func(uint8_t *data, uint32_t len)
{
	#if SDKUART
	if(data)
		free(data);
		data = NULL;
	#endif
}
/**@brief Function for handling the Write event.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_write(ble_sdk_t * p_lbs, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
	  int chunk_size = 0;
	#if SDKUART
		printf("\r\n\non_write\r\n");
	#endif
		if(g_rcv_data.offset == 0)
			{
				#if SDKUART
				g_rcv_data.data = (uint8_t *)malloc(SDK_INFO_LEN);	
				#endif
			}	
		//info write handler
    if ((p_evt_write->handle == p_lbs->sdk_info_char_handler.value_handle) &&
        (p_evt_write->len <= SDK_INFO_LEN) &&
        (p_lbs->sdk_info_handler != NULL))
    {
			chunk_size = p_evt_write->len > BLE_SDK_MAX_DATA_LEN ? BLE_SDK_MAX_DATA_LEN : p_evt_write->len;
			memcpy(g_rcv_data.data + g_rcv_data.offset, p_evt_write->data, chunk_size);
			g_rcv_data.offset += chunk_size;
			if(chunk_size < BLE_SDK_MAX_DATA_LEN || g_rcv_data.offset == 80){
					p_lbs->sdk_info_handler(p_lbs, g_rcv_data.data,g_rcv_data.offset);
				  SDK_data_free_func(g_rcv_data.data,g_rcv_data.offset);
				  g_rcv_data.offset = 0;
				#if SDKUART
				printf("\r\n\ninfo write handler\r\n");
				#endif
			}
        
    }
		
		//auth write handler
		if ((p_evt_write->handle == p_lbs->sdk_auth_char_handler.value_handle) &&
        (p_evt_write->len <= SDK_AUTH_LEN) &&
        (p_lbs->sdk_auth_handler != NULL))
    {
			chunk_size = p_evt_write->len > BLE_SDK_MAX_DATA_LEN ? BLE_SDK_MAX_DATA_LEN : p_evt_write->len;
			memcpy(g_rcv_data.data + g_rcv_data.offset, p_evt_write->data, chunk_size);
			g_rcv_data.offset += chunk_size;
			if(chunk_size < BLE_SDK_MAX_DATA_LEN || g_rcv_data.offset == 80){
					p_lbs->sdk_auth_handler(p_lbs, g_rcv_data.data,g_rcv_data.offset);
				  SDK_data_free_func(g_rcv_data.data,g_rcv_data.offset);
				  g_rcv_data.offset = 0;
				#if SDKUART
				printf("\r\n\nauth write handler\r\n");
				#endif
			}
        
    }
		//session write handler
		if ((p_evt_write->handle == p_lbs->sdk_session_char_handler.value_handle) &&
        (p_evt_write->len <= SDK_SESSION_LEN) &&
        (p_lbs->sdk_session_handler != NULL))
    {
			chunk_size = p_evt_write->len > BLE_SDK_MAX_DATA_LEN ? BLE_SDK_MAX_DATA_LEN : p_evt_write->len;
			memcpy(g_rcv_data.data + g_rcv_data.offset, p_evt_write->data, chunk_size);
			g_rcv_data.offset += chunk_size;
			if(chunk_size < BLE_SDK_MAX_DATA_LEN ){
					p_lbs->sdk_session_handler(p_lbs, g_rcv_data.data,g_rcv_data.offset);
				  SDK_data_free_func(g_rcv_data.data,g_rcv_data.offset);
				  g_rcv_data.offset = 0;
				#if SDKUART
				printf("\r\n\nsession write handler\r\n");
				#endif
			}
        
    }
		//cmd write handler
		if ((p_evt_write->handle == p_lbs->sdk_cmd_char_handler.value_handle) &&
        (p_evt_write->len <= SDK_CMD_LEN) &&
        (p_lbs->sdk_cmd_handler != NULL))
    {
			chunk_size = p_evt_write->len > BLE_SDK_MAX_DATA_LEN ? BLE_SDK_MAX_DATA_LEN : p_evt_write->len;
			memcpy(g_rcv_data.data + g_rcv_data.offset, p_evt_write->data, chunk_size);
			g_rcv_data.offset += chunk_size;
			if(chunk_size < BLE_SDK_MAX_DATA_LEN || g_rcv_data.offset == 20){
					p_lbs->sdk_cmd_handler(p_lbs, g_rcv_data.data,g_rcv_data.offset);
				  SDK_data_free_func(g_rcv_data.data,g_rcv_data.offset);
				  g_rcv_data.offset = 0;
				#if SDKUART
				printf("\r\n\ncmd write handler\r\n");
				#endif
			}
        
    }
		//rssi write handler
		if ((p_evt_write->handle == p_lbs->sdk_rssi_char_handler.value_handle) &&
        (p_evt_write->len <= SDK_RSSI_LEN) &&
        (p_lbs->sdk_rssi_handler != NULL))
    {
			chunk_size = p_evt_write->len > BLE_SDK_MAX_DATA_LEN ? BLE_SDK_MAX_DATA_LEN : p_evt_write->len;
			memcpy(g_rcv_data.data + g_rcv_data.offset, p_evt_write->data, chunk_size);
			g_rcv_data.offset += chunk_size;
			if(chunk_size < BLE_SDK_MAX_DATA_LEN ){
					p_lbs->sdk_rssi_handler(p_lbs, g_rcv_data.data,g_rcv_data.offset);
				  SDK_data_free_func(g_rcv_data.data,g_rcv_data.offset);
				  g_rcv_data.offset = 0;
				#if SDKUART
				printf("\r\n\nrssi write handler\r\n");
				#endif
			}
    }
}
void ble_sdk_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
	ble_sdk_t * p_sdk = (ble_sdk_t *)p_context;
	
	switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_sdk, p_ble_evt);
				#if SDKUART
						printf("\r\n\nBLE_GAP_EVT_CONNECTED\r\n");
				#endif
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_sdk, p_ble_evt);
				#if SDKUART
						printf("\r\n\nBLE_GAP_EVT_DISCONNECTED\r\n");
				#endif
            break;
            
        case BLE_GATTS_EVT_WRITE:
            on_write(p_sdk, p_ble_evt);
				#if SDKUART
						printf("\r\n\nBLE_GATTS_EVT_WRITE\r\n");
				#endif
            break;

        default:
            // No implementation needed.
            break;
    }
}
//void ble_sdk_on_ble_evt(ble_sdk_t * p_lbs, ble_evt_t * p_ble_evt)
//{
//    switch (p_ble_evt->header.evt_id)
//    {
//        case BLE_GAP_EVT_CONNECTED:
//            on_connect(p_lbs, p_ble_evt);
//            break;

//        case BLE_GAP_EVT_DISCONNECTED:
//            on_disconnect(p_lbs, p_ble_evt);
//            break;
//            
//        case BLE_GATTS_EVT_WRITE:
//            on_write(p_lbs, p_ble_evt);
//            break;

//        default:
//            // No implementation needed.
//            break;
//    }
//}

static uint32_t info_char_add(ble_sdk_t * p_lbs, const ble_sdk_init_t * p_lbs_init)
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

    ble_uuid.type = p_lbs->sdk_uuid_type;
    ble_uuid.uuid = INFO_UUID_CHAR;
    
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
		#if SDKUART
		printf("\r\ninfo_char_add.\r\n");
		#endif
    return sd_ble_gatts_characteristic_add(p_lbs->sdk_service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_lbs->sdk_info_char_handler);
}
static uint32_t auth_char_add(ble_sdk_t * p_lbs, const ble_sdk_init_t * p_lbs_init)
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

    ble_uuid.type = p_lbs->sdk_uuid_type;
    ble_uuid.uuid = AUTH_UUID_CHAR;
    
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
		#if SDKUART
		printf("\r\nauth_char_add.\r\n");
		#endif
    return sd_ble_gatts_characteristic_add(p_lbs->sdk_service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_lbs->sdk_auth_char_handler);
}
static uint32_t session_char_add(ble_sdk_t * p_lbs, const ble_sdk_init_t * p_lbs_init)
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

    ble_uuid.type = p_lbs->sdk_uuid_type;
    ble_uuid.uuid = SESSION_UUID_CHAR;
    
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
		#if SDKUART
		printf("\r\nsession_char_add.\r\n");
		#endif
    return sd_ble_gatts_characteristic_add(p_lbs->sdk_service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_lbs->sdk_session_char_handler);
}
static uint32_t cmd_char_add(ble_sdk_t * p_lbs, const ble_sdk_init_t * p_lbs_init)
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

    ble_uuid.type = p_lbs->sdk_uuid_type;
    ble_uuid.uuid = CMD_UUID_CHAR;
    
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
		#if SDKUART
		printf("\r\ncmd_char_add.\r\n");
		#endif
    return sd_ble_gatts_characteristic_add(p_lbs->sdk_service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_lbs->sdk_cmd_char_handler);
}
static uint32_t rssi_char_add(ble_sdk_t * p_lbs, const ble_sdk_init_t * p_lbs_init)
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

    ble_uuid.type = p_lbs->sdk_uuid_type;
    ble_uuid.uuid = RSSI_UUID_CHAR;
    
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
		#if SDKUART
		printf("\r\nrssi_char_add.\r\n");
		#endif
    return sd_ble_gatts_characteristic_add(p_lbs->sdk_service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_lbs->sdk_rssi_char_handler);
}
#if 1	
uint32_t ble_sdk_init(ble_sdk_t * p_lbs, const ble_sdk_init_t * p_lbs_init)
{

	  ret_code_t            err_code;
    ble_uuid_t            ble_uuid;
    ble_uuid128_t         sdk_base_uuid = SDK_UUID_BASE;
    ble_add_char_params_t add_char_params;
    VERIFY_PARAM_NOT_NULL(p_lbs);
    VERIFY_PARAM_NOT_NULL(p_lbs_init);
	
    // Initialize the service structure.
	#if SDKUART
    p_lbs->data_handler = p_lbs_init->data_handler;
	#endif
	
    /**@snippet [Adding proprietary Service to the SoftDevice] */
    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&sdk_base_uuid, &p_lbs->sdk_uuid_type);
    VERIFY_SUCCESS(err_code);
    ble_uuid.type = p_lbs->sdk_uuid_type;
    ble_uuid.uuid = SDK_UUID_SERVICE;
    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_lbs->sdk_service_handle);
    /**@snippet [Adding proprietary Service to the SoftDevice] */
    VERIFY_SUCCESS(err_code);
    // Add the INFO Characteristic.
		
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                     = INFO_UUID_CHAR;
    add_char_params.uuid_type                = p_lbs->sdk_uuid_type;
    add_char_params.max_len                  = SDK_INFO_LEN;
    add_char_params.init_len                 = sizeof(uint8_t);
    add_char_params.is_var_len               = true;
    add_char_params.char_props.write         = 1;
    add_char_params.char_props.write_wo_resp = 1;
    add_char_params.read_access  = SEC_OPEN;
    add_char_params.write_access = SEC_OPEN;
    err_code = characteristic_add(p_lbs->sdk_service_handle, &add_char_params, &p_lbs->sdk_info_char_handler);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    // Add the AUTH Characteristic.
    /**@snippet [Adding proprietary characteristic to the SoftDevice] */
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = AUTH_UUID_CHAR;
    add_char_params.uuid_type         = p_lbs->sdk_uuid_type;
    add_char_params.max_len           = SDK_AUTH_LEN;
    add_char_params.init_len          = sizeof(uint8_t);
    add_char_params.is_var_len        = true;
    add_char_params.char_props.notify = 1;
    add_char_params.read_access       = SEC_OPEN;
    add_char_params.write_access      = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;
    return characteristic_add(p_lbs->sdk_service_handle, &add_char_params, &p_lbs->sdk_auth_char_handler);
		
    /**@snippet [Adding proprietary characteristic to the SoftDevice] */
}
#endif
#if 0
uint32_t ble_sdk_init(ble_sdk_t * p_lbs, const ble_sdk_init_t * p_lbs_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure.
    p_lbs->sdk_conn_handle       = BLE_CONN_HANDLE_INVALID;
    p_lbs->data_handler = p_lbs_init->data_handler;
		p_lbs->sdk_info_handler = p_lbs_init->info_write_function_handler;
		p_lbs->sdk_auth_handler = p_lbs_init->auth_write_function_handler;
		p_lbs->sdk_session_handler = p_lbs_init->session_write_function_handler;
		p_lbs->sdk_cmd_handler = p_lbs_init->cmd_write_function_handler;
		p_lbs->sdk_rssi_handler = p_lbs_init->rssi_write_function_handler;

    // Add a custom base UUID.
    ble_uuid128_t base_uuid = {SDK_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_lbs->sdk_uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_lbs->sdk_uuid_type;
    ble_uuid.uuid = SDK_UUID_SERVICE;
   // Add the service.
		err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_lbs->sdk_service_handle);
    VERIFY_SUCCESS(err_code);

    // Add characteristics

		err_code = info_char_add(p_lbs, p_lbs_init);
    VERIFY_SUCCESS(err_code);
		
		err_code = auth_char_add(p_lbs, p_lbs_init);
    VERIFY_SUCCESS(err_code);
		
		err_code = session_char_add(p_lbs, p_lbs_init);
    VERIFY_SUCCESS(err_code);
		
		err_code = cmd_char_add(p_lbs, p_lbs_init);
    VERIFY_SUCCESS(err_code);
		
		err_code = rssi_char_add(p_lbs, p_lbs_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}
#endif

uint32_t ble_SDK_char_set(ble_sdk_t * p_lbs, uint16_t uuid, uint8_t * showValues, uint16_t valueLen)
{
	  ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, valueLen);

    gatts_value.len     = valueLen;
    gatts_value.offset  = 0;
    gatts_value.p_value = showValues;
	switch (uuid)
		{
			case	INFO_UUID_CHAR:
				return sd_ble_gatts_value_set(p_lbs->sdk_conn_handle, p_lbs->sdk_info_char_handler.value_handle, &gatts_value);
				//break;
			case	AUTH_UUID_CHAR:
				return sd_ble_gatts_value_set(p_lbs->sdk_conn_handle, p_lbs->sdk_auth_char_handler.value_handle, &gatts_value);
				//break;
			case	SESSION_UUID_CHAR:
				return sd_ble_gatts_value_set(p_lbs->sdk_conn_handle, p_lbs->sdk_session_char_handler.value_handle, &gatts_value);
				//break;
			case	CMD_UUID_CHAR:
				return sd_ble_gatts_value_set(p_lbs->sdk_conn_handle, p_lbs->sdk_cmd_char_handler.value_handle, &gatts_value);
				//break;
			case	RSSI_UUID_CHAR:
				return sd_ble_gatts_value_set(p_lbs->sdk_conn_handle, p_lbs->sdk_rssi_char_handler.value_handle, &gatts_value);
				//break;
			default:
				break;
		}	
		return 0;
}

