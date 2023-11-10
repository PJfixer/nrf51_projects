
#ifndef BLE_WEA_H__
#define BLE_WEA_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

//3be0f7d7-46da-40b2-b13f-915cca181e55 

#define WEA_UUID_BASE        {0x3b, 0xe0, 0xf7, 0xd7, 0x46, 0xda, 0x40, 0xb2, \
                              0xb1, 0x3f, 0x91, 0x5c, 0x00, 0x00, 0x00, 0x00}
#define WEA_UUID_SERVICE     0x1523
#define WEA_UUID_HUMI_CHAR 0x1524
#define WEA_UUID_TEMP_CHAR    0x1525
#define WEA_UUID_PRESS_CHAR 0x1527

#define WEA_UUID_NAME_CHAR 0x1528
#define NAME_CHAR_MAX_LEN 15
#define NAME_CHAR_DEFAULT "SENSORx"


// Forward declaration of the ble_WEA_t type.
typedef struct ble_WEA_s ble_WEA_t;

typedef void (*ble_WEA_write_handler_t) (ble_WEA_t * p_WEA, uint8_t new_state);



typedef struct
{
    ble_WEA_write_handler_t write_handler; 
} ble_WEA_init_t;  

/**@brief weather sensor  Service structure. This structure contains various status information for the service. */
struct ble_WEA_s
{
    uint16_t                    service_handle;      /**< Handle of weather sensor Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    name_char_handles; /**< Handles related to the pressure  Characteristic. */
    uint8_t                     uuid_type;           /**< UUID type for the Weather sensor Service. */
    uint16_t                    conn_handle;         /**< Handle of the current connection (as provided by the BLE stack). BLE_CONN_HANDLE_INVALID if not in a connection. */
    //ble_WEA_write_handler_t write_handler;   /**< Event handler to be called when the  Characteristic is written. NO USED IN THIS CASE */
};


uint32_t ble_WEA_init(ble_WEA_t * p_WEA);

/*
 * @param[in] p_WEA      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
void ble_WEA_on_ble_evt(ble_WEA_t * p_WEA, ble_evt_t * p_ble_evt);

/**@brief Function for sending a button state notification.
 *
 * @param[in] p_WEA      LED Button Service structure.
 * @param[in] button_state  New  state.
 *
 * @retval NRF_SUCCESS If the notification was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_WEA_on_change(ble_WEA_t * p_WEA, uint8_t new_state);

uint32_t ble_wea_name_value_update(ble_WEA_t * p_wea, char * custom_value,uint8_t len);

#ifdef __cplusplus
}
#endif

#endif // BLE_WEA_H__

/** @} */
