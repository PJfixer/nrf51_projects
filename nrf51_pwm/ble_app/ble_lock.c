
#include "sdk_common.h"
#include "ble_lock.h"
#include "ble_srv_common.h"
#include "flash.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"



static void on_connect(ble_lock_t * p_lock, ble_evt_t * p_ble_evt)
{
    p_lock->conn_handle = p_ble_evt->evt.gap_evt.conn_handle; // store the connection handle 
    NRF_LOG_INFO("lock on connect store handle %04x \n", p_lock->conn_handle);
        if(p_lock->connect_handler != NULL)
        {
            p_lock->connect_handler(p_lock);
        }
}


static void on_disconnect(ble_lock_t * p_lock, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_lock->conn_handle = BLE_CONN_HANDLE_INVALID;
    if(p_lock->disconnect_handler != NULL)
    {
            p_lock->disconnect_handler(p_lock);
    }
}


static void on_write(ble_lock_t * p_lock, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    NRF_LOG_INFO("on write lock ! handle : %04x , %04x \n",(p_evt_write->handle),p_lock->lock_char_handles.value_handle);
    if ((p_evt_write->handle == p_lock->lock_char_handles.value_handle) /*&& (p_evt_write->len <= LOCK_CHAR_MAX_LEN )*/)
    {
          NRF_LOG_INFO("on write lock 2 ! \n");
        if(p_lock->write_handler != NULL)
        {
            p_lock->write_handler(p_lock, p_evt_write->data[0]);
        }
    } 
}


void ble_lock_on_ble_evt(ble_lock_t * p_lock, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_lock, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_lock, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_lock, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}



static uint32_t lock_char_add(ble_lock_t * p_lock, const ble_lock_init_t * p_lock_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t lock_state = 0;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1; //  can't be written read only
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_lock->uuid_type; // copy uuid type
    ble_uuid.uuid = LOCK_UUID_LOCK_CHAR; // set uid 

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
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0; 
    attr_char_value.max_len      = sizeof(uint8_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_lock->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_lock->lock_char_handles);
}

uint32_t ble_lock_value_update(ble_lock_t * p_lock, char * custom_value,uint8_t len)
{
    if (p_lock == NULL)
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
    err_code = sd_ble_gatts_value_set(p_lock->conn_handle,
                                    p_lock->lock_char_handles.value_handle,
                                    &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

}




uint32_t ble_lock_init(ble_lock_t * p_lock,const ble_lock_init_t * p_lock_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure.
    p_lock->conn_handle       = BLE_CONN_HANDLE_INVALID;
    p_lock->write_handler = p_lock_init->write_handler;
    p_lock->connect_handler = p_lock_init->connect_handler;
    p_lock->disconnect_handler = p_lock_init->disconnect_handler;


    // Add service.
    ble_uuid128_t base_uuid = {LOCK_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_lock->uuid_type);
    NRF_LOG_INFO("error code sd_ble_uuid_vs_add : %d \n",err_code);
    VERIFY_SUCCESS(err_code);
 

    ble_uuid.type = p_lock->uuid_type;
    ble_uuid.uuid = LOCK_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_lock->service_handle); //add thez service
    VERIFY_SUCCESS(err_code); 



    err_code = lock_char_add(p_lock, NULL); 
    VERIFY_SUCCESS(err_code);  


    return NRF_SUCCESS;
}


