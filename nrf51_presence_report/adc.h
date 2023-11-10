
#ifndef ADC_H__
#define ADC_H__

#include "nrf.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "nrf_drv_adc.h"
#include "nordic_common.h"
#include "app_error.h"


#define ADC_CHAN_PIN NRF_ADC_CONFIG_INPUT_0

void adc_config(void);
uint16_t get_adc_value(void);

#endif