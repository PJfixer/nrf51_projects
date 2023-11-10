
#ifndef BLE_lock_H__
#define BLE_lock_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

//3be0f7d7-46da-40b2-b13f-915cca181e55 
//30 e6 c3 08-6c a7-42 b5-a7bc-964ae263484e
#define LOCK_UUID_BASE        {0x30, 0xe6, 0xc3, 0x08, 0x6c, 0xa7, 0x42, 0xb5, \
                              0xa7, 0xbc, 0x96, 0x4a, 0x00, 0x00, 0x00, 0x00}

#define LOCK_UUID_SERVICE     0x1535
#define LOCK_UUID_LOCK_CHAR 0x1536
#define LOCK_CHAR_MAX_LEN 1
#define LOCK_CHAR_DEFAULT "SENSORx"


// Forward declaration of the ble_lock_t type.
typedef struct ble_lock_s ble_lock_t;

typedef void (*ble_lock_write_handler_t) (ble_lock_t * p_nam, uint8_t new_state);
typedef void (*ble_lock_connect_handler_t) (ble_lock_t * p_nam);
typedef void (*ble_lock_disconnect_handler_t) (ble_lock_t * p_nam);

typedef struct
{
  ble_lock_write_handler_t write_handler; 
  ble_lock_connect_handler_t connect_handler;
  ble_lock_disconnect_handler_t disconnect_handler;
} ble_lock_init_t;  


struct ble_lock_s
{
    uint16_t                    service_handle;      /**< Handle of weather sensor Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    lock_char_handles;          /**< Handles related to the   Characteristic. */
    uint8_t                     uuid_type;           /**< UUID type for the Weather sensor Service. */
    uint16_t                    conn_handle;         /**< Handle of the current connection (as provided by the BLE stack). BLE_CONN_HANDLE_INVALID if not in a connection. */
    ble_lock_write_handler_t write_handler;   /**< Event handler to be called when the  Characteristic is written. NO USED IN THIS CASE */
    ble_lock_connect_handler_t connect_handler;
    ble_lock_disconnect_handler_t disconnect_handler;
};


uint32_t ble_lock_init(ble_lock_t * p_lock,const ble_lock_init_t * p_lock_init);

 
void ble_lock_on_ble_evt(ble_lock_t * p_lock, ble_evt_t * p_ble_evt);

 
uint32_t ble_lock_on_change(ble_lock_t * p_lock, uint8_t new_state);

uint32_t ble_lock_value_update(ble_lock_t * p_lock, char * custom_value,uint8_t len);

#ifdef __cplusplus
}
#endif

#endif // BLE_lock_H__


