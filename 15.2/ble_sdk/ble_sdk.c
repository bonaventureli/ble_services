#include "ble_sdk.h"
#include "ble_srv_common.h"
#include "sdk_common.h"

static data_info g_send_data = {NULL, 0, 0};
static data_info g_rcv_data = {NULL, 0, 0};

/**@brief Function for handling the Connect event.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_connect(ble_sdk_t * p_lbs, ble_evt_t * p_ble_evt)
{
    p_lbs->sdk_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}
/**@brief Function for handling the Disconnect event.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_disconnect(ble_sdk_t * p_lbs, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_lbs->sdk_conn_handle = BLE_CONN_HANDLE_INVALID;
}
void SDK_data_free_func(uint8_t *data, uint32_t len)
{
	if(data)
		free(data);
		data = NULL;
}
/**@brief Function for handling the Write event.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_write(ble_sdk_t * p_lbs, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
	  int chunk_size = 0;
	
		if(g_rcv_data.offset == 0)
			{
				g_rcv_data.data = (uint8_t *)malloc(SDK_INFO_LEN);	
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
					p_lbs->sdk_auth_handler(p_lbs, g_rcv_data.data,g_rcv_data.offset);
				  SDK_data_free_func(g_rcv_data.data,g_rcv_data.offset);
				  g_rcv_data.offset = 0;
			}
    }
}

void ble_sdk_on_ble_evt(ble_sdk_t * p_lbs, ble_evt_t * p_ble_evt)
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

    return sd_ble_gatts_characteristic_add(p_lbs->sdk_service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_lbs->sdk_rssi_char_handler);
}

uint32_t ble_sdk_init(ble_sdk_t * p_lbs, const ble_sdk_init_t * p_lbs_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure.初始化服务结构
    p_lbs->sdk_conn_handle       = BLE_CONN_HANDLE_INVALID;// 初始化连接句柄，因为现在并未与手机连接所以先赋值无效。
    
		p_lbs->sdk_info_handler = p_lbs_init->info_write_function_handler;
		p_lbs->sdk_auth_handler = p_lbs_init->auth_write_function_handler;
		p_lbs->sdk_session_handler = p_lbs_init->session_write_function_handler;
		p_lbs->sdk_cmd_handler = p_lbs_init->cmd_write_function_handler;
		p_lbs->sdk_rssi_handler = p_lbs_init->rssi_write_function_handler;

    // Add service.
    ble_uuid128_t base_uuid = {SDK_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_lbs->sdk_uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_lbs->sdk_uuid_type;
    ble_uuid.uuid = SDK_UUID_SERVICE;
   //GATT服务添加，主要是添加服务UUID
		err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_lbs->sdk_service_handle);
		
    VERIFY_SUCCESS(err_code);

    // Add characteristics.添加特征值

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

