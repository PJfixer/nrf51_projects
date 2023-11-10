
#include "sdk_common.h"
#include "ble_nam.h"
#include "ble_srv_common.h"
#include "flash.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"



static void on_connect(ble_nam_t * p_nam, ble_evt_t * p_ble_evt)
{
    p_nam->conn_handle = p_ble_evt->evt.gap_evt.conn_handle; // store the connection handle 
    NRF_LOG_INFO("nam on connect store handle %04x \n", p_nam->conn_handle);
}


static void on_disconnect(ble_nam_t * p_nam, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_nam->conn_handle = BLE_CONN_HANDLE_INVALID;
}


static void on_write(ble_nam_t * p_nam, ble_evt_t * p_ble_evt)
{
    
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
     NRF_LOG_INFO("on write nam ! handle : %04x , %04x  \n",(p_evt_write->handle),p_nam->name_char_handles.value_handle);
    if ((p_evt_write->handle == p_nam->name_char_handles.value_handle) && (p_evt_write->len > 0 && p_evt_write->len <= NAME_CHAR_MAX_LEN ))
    {
       // ble_nam_name_value_update(p_nam,&p_evt_write->data[0],p_evt_write->len);
        NRF_LOG_INFO("writing in flash... %d bytes ",p_evt_write->len);
        update_name_flash(&p_evt_write->data[0],p_evt_write->len);
        NRF_LOG_INFO("done ! \n ");
    } 
}


void ble_nam_on_ble_evt(ble_nam_t * p_nam, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_nam, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_nam, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_nam, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}



static uint32_t name_char_add(ble_nam_t * p_nam, const ble_nam_init_t * p_nam_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1; // humi can't be written read only
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_nam->uuid_type; // copy uuid type
    ble_uuid.uuid = NAM_UUID_NAME_CHAR; // set uid 

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm); // init read perm
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);// init write perm

    attr_md.vloc       = BLE_GATTS_VLOC_STACK; // don't know ...
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid; // fill the ble_gatts_attr_t struct before softdevice for add the characteristic 
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = NAME_CHAR_MAX_LEN;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = NAME_CHAR_MAX_LEN;
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_nam->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_nam->name_char_handles);
}

uint32_t ble_nam_name_value_update(ble_nam_t * p_nam, char * custom_value,uint8_t len)
{
    if (p_nam == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = len;
    gatts_value.offset  = 0;
    gatts_value.p_value = custom_value;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_nam->conn_handle,
                                    p_nam->name_char_handles.value_handle,
                                    &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

}




uint32_t ble_nam_init(ble_nam_t * p_nam)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure.
    p_nam->conn_handle       = BLE_CONN_HANDLE_INVALID;
   // p_nam->led_write_handler = p_nam_init->led_write_handler; // at init copy callback ptr needed for write event 

    // Add service.
    ble_uuid128_t base_uuid = {NAM_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_nam->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_nam->uuid_type;
    ble_uuid.uuid = NAM_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_nam->service_handle); //add thez service
    VERIFY_SUCCESS(err_code);



    err_code = name_char_add(p_nam, NULL); 
    VERIFY_SUCCESS(err_code);  



    return NRF_SUCCESS;
}

//example of notify , char ccmd should be set in order to be able to notify
/*uint32_t ble_nam_on_button_change(ble_nam_t * p_nam, uint8_t button_state)
{
    ble_gatts_hvx_params_t params;
    uint16_t len = sizeof(button_state);

    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_nam->button_char_handles.value_handle;
    params.p_data = &button_state;
    params.p_len = &len;

    return sd_ble_gatts_hvx(p_nam->conn_handle, &params);
} */ 
 // NRF_MODULE_ENABLED(BLE_nam)
