
#include "adc.h"

 nrf_drv_adc_channel_t m_channel_config = NRF_DRV_ADC_DEFAULT_CHANNEL(ADC_CHAN_PIN);

void adc_config(void)
{
 
    nrf_drv_adc_channel_enable(&m_channel_config);
}


uint16_t get_adc_value(void)
{
      ret_code_t ret_code;
      nrf_adc_value_t  adc_val;
      nrf_drv_adc_sample_convert(&m_channel_config,&adc_val);
      APP_ERROR_CHECK(ret_code);
      return (uint16_t)adc_val;
}