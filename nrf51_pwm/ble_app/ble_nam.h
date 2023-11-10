
#ifndef BLE_nam_H__
#define BLE_nam_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

//3be0f7d7-46da-40b2-b13f-915cca181e55 

#define NAM_UUID_BASE        {0x3b, 0xe0, 0xf7, 0xd7, 0x46, 0xda, 0x40, 0xb2, \
                              0xb1, 0x3f, 0x91, 0x5c, 0x00, 0x00, 0x00, 0x00}

#define NAM_UUID_SERVICE     0x1523
#define NAM_UUID_NAME_CHAR 0x1528
#define NAME_CHAR_MAX_LEN 15
#define NAME_CHAR_DEFAULT "SENSORx"


// Forward declaration of the ble_nam_t type.
typedef struct ble_nam_s ble_nam_t;

typedef void (*ble_nam_write_handler_t) (ble_nam_t * p_nam, uint8_t new_state);


typedef struct
{
    ble_nam_write_handler_t write_handler; 
} ble_nam_init_t;  


struct ble_nam_s
{
    uint16_t                    service_handle;      /**< Handle of weather sensor Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    name_char_handles; /**< Handles related to the pressure  Characteristic. */
    uint8_t                     uuid_type;           /**< UUID type for the Weather sensor Service. */
    uint16_t                    conn_handle;         /**< Handle of the current connection (as provided by the BLE stack). BLE_CONN_HANDLE_INVALID if not in a connection. */
    //ble_nam_write_handler_t write_handler;   /**< Event handler to be called when the  Characteristic is written. NO USED IN THIS CASE */
};


uint32_t ble_nam_init(ble_nam_t * p_nam);

 
void ble_nam_on_ble_evt(ble_nam_t * p_nam, ble_evt_t * p_ble_evt);

 
uint32_t ble_nam_on_change(ble_nam_t * p_nam, uint8_t new_state);

uint32_t ble_nam_name_value_update(ble_nam_t * p_nam, char * custom_value,uint8_t len);

#ifdef __cplusplus
}
#endif

#endif // BLE_nam_H__


