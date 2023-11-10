

#include "app_button.h"
#include "app_error.h"
#include "app_timer.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "ble_lbs.h"
#include "ble_lock.h"
#include "ble_nam.h"
#include "ble_srv_common.h"
#include "bsp.h"
#include "flash.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_timer.h"
#include "softdevice_handler.h"
#include <stdint.h>
#include <string.h>

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define CENTRAL_LINK_COUNT                                                                                             \
    0 /**< Number of central links used by the application. When changing this number remember to adjust the RAM       \
         settings*/
#define PERIPHERAL_LINK_COUNT                                                                                          \
    1 /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM    \
         settings*/

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE                                                                                           \
    GATT_MTU_SIZE_DEFAULT /**< MTU size used in the softdevice enabling and to reply to a                              \
                             BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED                                                                                      \
    BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2 /**< Reply when unsupported features are requested. */

#define DEVICE_NAME "Pjfixer_DETEC1" /**< Name of device. Will be included in the advertising data. */

#define APP_ADV_INTERVAL                                                                                               \
    4000 /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms).                        \
          */
#define APP_ADV_TIMEOUT_IN_SECONDS                                                                                     \
    0 /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

#define APP_TIMER_PRESCALER 0     /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS 6    /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE 4 /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL                                                                                              \
    MSEC_TO_UNITS(100, UNIT_1_25_MS)                       /**< Minimum acceptable connection interval (0.5 seconds).  \
                                                            */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(200, UNIT_1_25_MS) /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY 0                                    /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)   /**< Connection supervisory time-out (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY                                                                                 \
    APP_TIMER_TICKS(20000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to  \
                                                   first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY                                                                                  \
    APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the   \
                                                  first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT                                                                                   \
    3 /**< Number of attempts before giving up the connection parameter negotiation.                                   \
       */

APP_TIMER_DEF(m_repeated_timer_id);
bool time_up_flag = false;

static void power_manage(void);

#define DEAD_BEEF                                                                                                      \
    0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define NB_PWM 6

#define MOSFET_POWER_PIN 16

#define PWM_PIN1 23

#define PWM_PIN2 24

#define PWM_PIN3 25

#define PWM_PIN4 28

#define PWM_PIN5 29

#define PWM_PIN6 30

#define UNLOCK_PULSE 26
#define LOCK_PULSE 51

enum machine_state
{
    IDLE,
    PROCESSING
} m_state;
uint8_t last_command = 0;
uint8_t steps_index = 0;

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */

static ble_nam_t m_nam_s;

static ble_lock_t m_lock_s;

uint8_t adata[31];
uint8_t dev_name[20] = "NEW_SENSORx";

const nrf_drv_timer_t TIMER_PWM = NRF_DRV_TIMER_INSTANCE(2);

uint32_t pwm_counter = 0;
uint16_t pwm_duties[NB_PWM] = {UNLOCK_PULSE, UNLOCK_PULSE, UNLOCK_PULSE, UNLOCK_PULSE, UNLOCK_PULSE, UNLOCK_PULSE};
uint8_t pwm_pins[NB_PWM] = {PWM_PIN1, PWM_PIN2, PWM_PIN3, PWM_PIN4, PWM_PIN5, PWM_PIN6};

void timer_event_handler(nrf_timer_event_t event_type, void *p_context)
{

    switch (event_type)
    {
    case NRF_TIMER_EVENT_COMPARE0:
        if (pwm_counter > 512)
        {
            pwm_counter = 0;
        }
        for (int i = 0; i < NB_PWM; i++)
        {
            if (pwm_counter >= pwm_duties[i])
            {
                nrf_gpio_pin_clear(pwm_pins[i]);
            }
            else
            {
                nrf_gpio_pin_set(pwm_pins[i]);
            }
        }
        pwm_counter++;
        break;

    default:
        // Do nothing.
        break;
    }
}

static void app_timer_handler(void *p_param)
{
    uint32_t err_code;
    time_up_flag = true;
    NRF_LOG_INFO("time up !! \n");

    APP_ERROR_CHECK(err_code);
}

void state_machine(void)
{
    switch (m_state)
    {
    case IDLE:
        power_manage();
        break;
    case PROCESSING:

        switch (last_command)
        {
        case 0xFF:
            switch (steps_index)
            {
            case 0: // steps 0 set power
                nrf_gpio_pin_set(MOSFET_POWER_PIN);
                steps_index++;
                set_ms_timer(300);

                break;
            case 1: // steps 1 set UNLOCK drawer 1
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("Unlocking Drawer 1...\r\n");
                    pwm_duties[0] = UNLOCK_PULSE; // 1000ms unlock D1
                    steps_index++;
                    set_ms_timer(300);
                }
                break;
            case 2: // steps 2 set UNLOCK drawer 1
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("Unlocking Drawer 2...\r\n");
                    pwm_duties[1] = UNLOCK_PULSE; // 1000ms unlock D2
                    steps_index++;
                    set_ms_timer(300);
                }
                break;
            case 3: // steps 3 set UNLOCK drawer 3
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("Unlocking Drawer 3...\r\n");
                    pwm_duties[2] = UNLOCK_PULSE; // 1000ms unlock D3
                    steps_index++;
                    set_ms_timer(300);
                }
                break;
            case 4: // steps 4 set UNLOCK drawer 4
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("Unlocking Drawer 4...\r\n");
                    pwm_duties[3] = UNLOCK_PULSE; // 1000ms unlock D4
                    steps_index++;
                    set_ms_timer(300);
                }
                break;
            case 5: // steps 5 set UNLOCK drawer 5
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("Unlocking Drawer 5...\r\n");
                    pwm_duties[4] = UNLOCK_PULSE; // 1000ms unlock D5
                    steps_index++;
                    set_ms_timer(300);
                }
                break;
            case 6: // steps 6 set UNLOCK drawer 6
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("Unlocking Drawer 6...\r\n");
                    pwm_duties[5] = UNLOCK_PULSE; // 1000ms unlock D6
                    steps_index++;
                    set_ms_timer(300);
                }
                break;
            case 7: // steps 7 cut power & back to IDLE
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("back to IDLE \n");
                    nrf_gpio_pin_clear(MOSFET_POWER_PIN);
                    m_state = IDLE;
                    steps_index = 0;
                    last_command = 0;
                }
                break;
            default:
                break;
            }

            break;

        case 0xEE:
            switch (steps_index)
            {
            case 0: // steps 0 set power
                nrf_gpio_pin_set(MOSFET_POWER_PIN);
                steps_index++;
                set_ms_timer(300);

                break;
            case 1: // steps 1 set LOCK drawer 1
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("Locking Drawer 1...\r\n");
                    pwm_duties[0] = LOCK_PULSE; // 1000ms lock D1
                    steps_index++;
                    set_ms_timer(300);
                }
                break;
            case 2: // steps 2 set LOCK drawer 1
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("Locking Drawer 2...\r\n");
                    pwm_duties[1] = LOCK_PULSE; // 1000ms lock D2
                    steps_index++;
                    set_ms_timer(300);
                }
                break;
            case 3: // steps 3 set LOCK drawer 3
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("Locking Drawer 3...\r\n");
                    pwm_duties[2] = LOCK_PULSE; // 1000ms lock D3
                    steps_index++;
                    set_ms_timer(300);
                }
                break;
            case 4: // steps 4 set LOCK drawer 4
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("Locking Drawer 4...\r\n");
                    pwm_duties[3] = LOCK_PULSE; // 1000ms lock D4
                    steps_index++;
                    set_ms_timer(300);
                }
                break;
            case 5: // steps 5 set LOCK drawer 5
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("Locking Drawer 5...\r\n");
                    pwm_duties[4] = LOCK_PULSE; // 1000ms lock D5
                    steps_index++;
                    set_ms_timer(300);
                }
                break;
            case 6: // steps 6 set LOCK drawer 6
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("Locking Drawer 6...\r\n");
                    pwm_duties[5] = LOCK_PULSE; // 1000ms lock D6
                    steps_index++;
                    set_ms_timer(300);
                }
                break;
            case 7: // steps 7 cut power & back to IDLE
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("back to IDLE \n");
                    nrf_gpio_pin_clear(MOSFET_POWER_PIN);
                    m_state = IDLE;
                    steps_index = 0;
                    last_command = 0;
                }
                break;
            default:
                break;
            }

            break;

        case 0xE1:
            switch (steps_index)
            {
            case 0: // steps 0 set power
                nrf_gpio_pin_set(MOSFET_POWER_PIN);
                steps_index++;
                set_ms_timer(300);

                break;
            case 1: // steps 1 set LOCK drawer 1
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("Locking Drawer 1...\r\n");
                    pwm_duties[0] = LOCK_PULSE; // 1000ms lock D1
                    steps_index++;
                    set_ms_timer(300);
                }
                break;
            case 2: // steps 2 cut power & back to IDLE
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("back to IDLE \n");
                    nrf_gpio_pin_clear(MOSFET_POWER_PIN);
                    m_state = IDLE;
                    steps_index = 0;
                    last_command = 0;
                }
                break;
            default:
                break;
            }

            break;

        case 0xE2:
            switch (steps_index)
            {
            case 0: // steps 0 set power
                nrf_gpio_pin_set(MOSFET_POWER_PIN);
                steps_index++;
                set_ms_timer(300);

                break;
            case 1: // steps 1 set LOCK drawer 1
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("Locking Drawer 2...\r\n");
                    pwm_duties[1] = LOCK_PULSE; // 1000ms lock D1
                    steps_index++;
                    set_ms_timer(300);
                }
                break;
            case 2: // steps 2 cut power & back to IDLE
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("back to IDLE \n");
                    nrf_gpio_pin_clear(MOSFET_POWER_PIN);
                    m_state = IDLE;
                    steps_index = 0;
                    last_command = 0;
                }
                break;
            default:
                break;
            }

            break;

        case 0xE3:
            switch (steps_index)
            {
            case 0: // steps 0 set power
                nrf_gpio_pin_set(MOSFET_POWER_PIN);
                steps_index++;
                set_ms_timer(300);

                break;
            case 1: // steps 1 set LOCK drawer 1
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("Locking Drawer 3...\r\n");
                    pwm_duties[2] = LOCK_PULSE; // 1000ms lock D1
                    steps_index++;
                    set_ms_timer(300);
                }
                break;
            case 2: // steps 2 cut power & back to IDLE
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("back to IDLE \n");
                    nrf_gpio_pin_clear(MOSFET_POWER_PIN);
                    m_state = IDLE;
                    steps_index = 0;
                    last_command = 0;
                }
                break;
            default:
                break;
            }

            break;

        case 0xE4:
            switch (steps_index)
            {
            case 0: // steps 0 set power
                nrf_gpio_pin_set(MOSFET_POWER_PIN);
                steps_index++;
                set_ms_timer(300);

                break;
            case 1: // steps 1 set LOCK drawer 4
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("Locking Drawer 4...\r\n");
                    pwm_duties[3] = LOCK_PULSE; // 1000ms lock D1
                    steps_index++;
                    set_ms_timer(300);
                }
                break;
            case 2: // steps 2 cut power & back to IDLE
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("back to IDLE \n");
                    nrf_gpio_pin_clear(MOSFET_POWER_PIN);
                    m_state = IDLE;
                    steps_index = 0;
                    last_command = 0;
                }
                break;
            default:
                break;
            }

            break;

        case 0xE5:
            switch (steps_index)
            {
            case 0: // steps 0 set power
                nrf_gpio_pin_set(MOSFET_POWER_PIN);
                steps_index++;
                set_ms_timer(300);

                break;
            case 1: // steps 1 set LOCK drawer 1
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("Locking Drawer 5...\r\n");
                    pwm_duties[4] = LOCK_PULSE; // 1000ms lock D1
                    steps_index++;
                    set_ms_timer(300);
                }
                break;
            case 2: // steps 2 cut power & back to IDLE
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("back to IDLE \n");
                    nrf_gpio_pin_clear(MOSFET_POWER_PIN);
                    m_state = IDLE;
                    steps_index = 0;
                    last_command = 0;
                }
                break;
            default:
                break;
            }

            break;

        case 0xE6:
            switch (steps_index)
            {
            case 0: // steps 0 set power
                nrf_gpio_pin_set(MOSFET_POWER_PIN);
                steps_index++;
                set_ms_timer(300);

                break;
            case 1: // steps 1 set LOCK drawer 1
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("Locking Drawer 6...\r\n");
                    pwm_duties[5] = LOCK_PULSE; // 1000ms lock D1
                    steps_index++;
                    set_ms_timer(300);
                }
                break;
            case 2: // steps 2 cut power & back to IDLE
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("back to IDLE \n");
                    nrf_gpio_pin_clear(MOSFET_POWER_PIN);
                    m_state = IDLE;
                    steps_index = 0;
                    last_command = 0;
                }
                break;
            default:
                break;
            }

            break;

        case 0xF1:
            switch (steps_index)
            {
            case 0: // steps 0 set power
                nrf_gpio_pin_set(MOSFET_POWER_PIN);
                steps_index++;
                set_ms_timer(300);

                break;
            case 1: // steps 1 set UNLOCK drawer 1
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("Unlocking Drawer 1...\r\n");
                    pwm_duties[0] = UNLOCK_PULSE; // 1000ms lock D1
                    steps_index++;
                    set_ms_timer(300);
                }
                break;
            case 2: // steps 2 cut power & back to IDLE
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("back to IDLE \n");
                    nrf_gpio_pin_clear(MOSFET_POWER_PIN);
                    m_state = IDLE;
                    steps_index = 0;
                    last_command = 0;
                }
                break;
            default:
                break;
            }

            break;

        case 0xF2:
            switch (steps_index)
            {
            case 0: // steps 0 set power
                nrf_gpio_pin_set(MOSFET_POWER_PIN);
                steps_index++;
                set_ms_timer(300);

                break;
            case 1: // steps 1 set UNLOCK drawer 1
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("Unlocking Drawer 2...\r\n");
                    pwm_duties[1] = UNLOCK_PULSE; // 1000ms lock D1
                    steps_index++;
                    set_ms_timer(300);
                }
                break;
            case 2: // steps 2 cut power & back to IDLE
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("back to IDLE \n");
                    nrf_gpio_pin_clear(MOSFET_POWER_PIN);
                    m_state = IDLE;
                    steps_index = 0;
                    last_command = 0;
                }
                break;
            default:
                break;
            }

            break;

        case 0xF3:
            switch (steps_index)
            {
            case 0: // steps 0 set power
                nrf_gpio_pin_set(MOSFET_POWER_PIN);
                steps_index++;
                set_ms_timer(300);

                break;
            case 1: // steps 1 set UNLOCK drawer 1
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("Unlocking Drawer 3...\r\n");
                    pwm_duties[2] = UNLOCK_PULSE; // 1000ms lock D1
                    steps_index++;
                    set_ms_timer(300);
                }
                break;
            case 2: // steps 2 cut power & back to IDLE
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("back to IDLE \n");
                    nrf_gpio_pin_clear(MOSFET_POWER_PIN);
                    m_state = IDLE;
                    steps_index = 0;
                    last_command = 0;
                }
                break;
            default:
                break;
            }

            break;

        case 0xF4:
            switch (steps_index)
            {
            case 0: // steps 0 set power
                nrf_gpio_pin_set(MOSFET_POWER_PIN);
                steps_index++;
                set_ms_timer(300);

                break;
            case 1: // steps 1 set UNLOCK drawer 4
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("Unlocking Drawer 4...\r\n");
                    pwm_duties[3] = UNLOCK_PULSE; // 1000ms lock D1
                    steps_index++;
                    set_ms_timer(300);
                }
                break;
            case 2: // steps 2 cut power & back to IDLE
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("back to IDLE \n");
                    nrf_gpio_pin_clear(MOSFET_POWER_PIN);
                    m_state = IDLE;
                    steps_index = 0;
                    last_command = 0;
                }
                break;
            default:
                break;
            }

            break;

        case 0xF5:
            switch (steps_index)
            {
            case 0: // steps 0 set power
                nrf_gpio_pin_set(MOSFET_POWER_PIN);
                steps_index++;
                set_ms_timer(300);

                break;
            case 1: // steps 1 set UNLOCK drawer 1
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("Unlocking Drawer 5...\r\n");
                    pwm_duties[4] = UNLOCK_PULSE; // 1000ms lock D1
                    steps_index++;
                    set_ms_timer(300);
                }
                break;
            case 2: // steps 2 cut power & back to IDLE
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("back to IDLE \n");
                    nrf_gpio_pin_clear(MOSFET_POWER_PIN);
                    m_state = IDLE;
                    steps_index = 0;
                    last_command = 0;
                }
                break;
            default:
                break;
            }

            break;

        case 0xF6:
            switch (steps_index)
            {
            case 0: // steps 0 set power
                nrf_gpio_pin_set(MOSFET_POWER_PIN);
                steps_index++;
                set_ms_timer(300);

                break;
            case 1: // steps 1 set UNLOCK drawer 1
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("Unlocking Drawer 6...\r\n");
                    pwm_duties[5] = UNLOCK_PULSE; // 1000ms lock D1
                    steps_index++;
                    set_ms_timer(300);
                }
                break;
            case 2: // steps 2 cut power & back to IDLE
                if (time_up_flag == true)
                {
                    NRF_LOG_INFO("back to IDLE \n");
                    nrf_gpio_pin_clear(MOSFET_POWER_PIN);
                    m_state = IDLE;
                    steps_index = 0;
                    last_command = 0;
                }
                break;
            default:
                break;
            }

            break;
        }
    }
}

void setup_hw_tim0(void)
{

    uint32_t time_ticks;
    uint32_t err_code = NRF_SUCCESS;

    // Configure TIMER_PWM for generating simple light effect - leds on board will invert his state one after the other.
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.frequency = NRF_TIMER_FREQ_1MHz;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_16;

    err_code = nrf_drv_timer_init(&TIMER_PWM, &timer_cfg, timer_event_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrf_drv_timer_us_to_ticks(&TIMER_PWM, 39);

    nrf_drv_timer_extended_compare(&TIMER_PWM, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   true);
}

void set_ms_timer(uint16_t delay_ms)
{

    uint32_t err_code;
    time_up_flag = false;
    err_code = app_timer_start(m_repeated_timer_id, APP_TIMER_TICKS(delay_ms, APP_TIMER_PRESCALER), NULL);
    NRF_LOG_INFO("%d", err_code);
    APP_ERROR_CHECK(err_code);
}

void gpio_pwm_init(void)
{
    nrf_gpio_cfg_output(MOSFET_POWER_PIN);
    nrf_gpio_pin_clear(MOSFET_POWER_PIN);
    nrf_gpio_cfg_output(PWM_PIN1);
    nrf_gpio_cfg_output(PWM_PIN2);
    nrf_gpio_cfg_output(PWM_PIN3);
    nrf_gpio_cfg_output(PWM_PIN4);
    nrf_gpio_cfg_output(PWM_PIN5);
    nrf_gpio_cfg_output(PWM_PIN6);
}

void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

void go_sleep(void)
{
    sd_power_system_off();
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    uint32_t err_code;
    err_code = app_timer_create(&m_repeated_timer_id, APP_TIMER_MODE_SINGLE_SHOT, app_timer_handler);
    APP_ERROR_CHECK(err_code);
}

static void gap_params_init(void)
{
    uint32_t err_code;
    ble_gap_conn_params_t gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

static void lock_state_connect_handler(ble_lock_t *p_lock)
{
    NRF_LOG_INFO("lock on connect enable PWM \r\n");
    pwm_counter = 0;
    nrf_drv_timer_enable(&TIMER_PWM);
}

static void lock_state_disconnect_handler(ble_lock_t *p_lock)
{
    NRF_LOG_INFO("lock on disconnect disable PWM \r\n");
    nrf_drv_timer_disable(&TIMER_PWM);
    nrf_gpio_pin_clear(PWM_PIN1);
    nrf_gpio_pin_clear(PWM_PIN2);
    nrf_gpio_pin_clear(PWM_PIN3);
    nrf_gpio_pin_clear(PWM_PIN4);
    nrf_gpio_pin_clear(PWM_PIN5);
    nrf_gpio_pin_clear(PWM_PIN6);
    pwm_counter = 0;
}

static void lock_state_write_handler(ble_lock_t *p_lock, uint8_t command)
{
    NRF_LOG_INFO("Received cmd  %d \r\n", command);
    last_command = command;
    m_state = PROCESSING;
}

static void services_init(void)
{
    uint32_t err_code;
    ble_lock_init_t init_lock;
    init_lock.write_handler = lock_state_write_handler;
    init_lock.connect_handler = lock_state_connect_handler;
    init_lock.disconnect_handler = lock_state_disconnect_handler;

    err_code = ble_nam_init(&m_nam_s);
    APP_ERROR_CHECK(err_code);
    err_code = ble_lock_init(&m_lock_s, &init_lock);
    APP_ERROR_CHECK(err_code);
}

static void on_conn_params_evt(ble_conn_params_evt_t *p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void conn_params_init(void)
{
    uint32_t err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail = false;
    cp_init.evt_handler = on_conn_params_evt;
    cp_init.error_handler = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

static void advertising_init(void)
{
    uint32_t err_code;

    memset(adata, 0x00, sizeof(adata));
    adata[0] = 30;
    adata[1] = BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA;

    // set PJFIXER IDENTIFIER
    adata[2] = 0xEF;
    adata[3] = 0xAD;

    // set device type 0xF5 = PJFIXER LOCK SYSTEM
    adata[4] = 0xF5;

    err_code = sd_ble_gap_adv_data_set(adata, sizeof(adata), NULL, 0);
    APP_ERROR_CHECK(err_code);
}

static void advertising_start(void)
{
    uint32_t err_code;
    ble_gap_adv_params_t adv_params;

    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp = BLE_GAP_ADV_FP_ANY;
    adv_params.interval = APP_ADV_INTERVAL;
    adv_params.timeout = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
}

static void on_ble_evt(ble_evt_t *p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        NRF_LOG_INFO("Connected\r\n");

        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

        break; // BLE_GAP_EVT_CONNECTED

    case BLE_GAP_EVT_DISCONNECTED:
        NRF_LOG_INFO("Disconnected\r\n");

        m_conn_handle = BLE_CONN_HANDLE_INVALID;

        APP_ERROR_CHECK(err_code);

        advertising_start();
        break; // BLE_GAP_EVT_DISCONNECTED

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        // Pairing not supported
        err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
        APP_ERROR_CHECK(err_code);
        break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        // No system attributes have been stored.
        err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
        APP_ERROR_CHECK(err_code);
        break; // BLE_GATTS_EVT_SYS_ATTR_MISSING

    case BLE_GATTC_EVT_TIMEOUT:
        // Disconnect on GATT Client timeout event.
        NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
        err_code =
            sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break; // BLE_GATTC_EVT_TIMEOUT

    case BLE_GATTS_EVT_TIMEOUT:
        // Disconnect on GATT Server timeout event.
        NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
        err_code =
            sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break; // BLE_GATTS_EVT_TIMEOUT

    case BLE_EVT_USER_MEM_REQUEST:
        err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
        APP_ERROR_CHECK(err_code);
        break; // BLE_EVT_USER_MEM_REQUEST

    case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST: {
        ble_gatts_evt_rw_authorize_request_t req;
        ble_gatts_rw_authorize_reply_params_t auth_reply;

        req = p_ble_evt->evt.gatts_evt.params.authorize_request;

        if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
        {
            if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ) ||
                (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
            {
                if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                {
                    auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                }
                else
                {
                    auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                }
                auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle, &auth_reply);
                APP_ERROR_CHECK(err_code);
            }
        }
    }
    break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
    case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
        err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle, NRF_BLE_MAX_MTU_SIZE);
        APP_ERROR_CHECK(err_code);
        break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

    default:
        // No implementation needed.
        break;
    }
}

static void ble_evt_dispatch(ble_evt_t *p_ble_evt)
{
    on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_lock_on_ble_evt(&m_lock_s, p_ble_evt);
    ble_nam_on_ble_evt(&m_nam_s, p_ble_evt);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT, &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // MAJOR CRICTICAL IMPORTANT FUCK !!! if u want multiple service u need to set vs_uuid_count to 2 or more
    ble_enable_params.common_enable_params.vs_uuid_count = 2;

    // Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Power Manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();

    APP_ERROR_CHECK(err_code);
}

/**@brief Function for application main entry.
 */
int main(void)
{
    ret_code_t err_code;

    timers_init();
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    ble_stack_init();
    gap_params_init();
    services_init();
    uint8_t name_len = get_name_flash(&dev_name[0]);
    NRF_LOG_INFO("len : %d \r\n", name_len);
    if (name_len != 0) // if flash is written use flash as device name however use default name
    {
        NRF_LOG_INFO("use flash stored dev name !\r\n");
        memcpy(&adata[12], &dev_name[0], name_len - 1);
    }

    advertising_init();
    conn_params_init();

    // Start execution.
    NRF_LOG_INFO(" Start!\r\n");
    advertising_start();
    gpio_pwm_init();
    setup_hw_tim0();

    // Enter main loop.
    for (;;)
    {
        if (NRF_LOG_PROCESS() == false)
        {
            // //set_ms_timer(1200);
            state_machine();
            // power_manage();
        }
    }
}

/**
 * @}
 */
