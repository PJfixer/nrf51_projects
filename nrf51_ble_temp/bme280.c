
#include "bme280.h"

struct BME280_Calibration_Data cal_data ; 
int32_t   t_fine;

extern const nrf_drv_twi_t m_twi;

 uint8_t BME280_read8(uint8_t reg)
{
   /* uint8_t temp ; 
    I2C_Master_Start();
    I2C_Master_Write(BME280_ADDRESS_W); //selection egistre
    I2C_Master_Wait();
    I2C_Master_Write(reg); //selection registre
    I2C_Master_Wait();
    I2C_Master_Stop();
    I2C_Master_Start();
    I2C_Master_Write(BME280_ADDRESS_R); //lecture
    I2C_Master_Wait();
    temp = I2C_Master_Read(I2C_NACK);
    I2C_Master_Wait();
    I2C_Master_Stop();*/
    ret_code_t ret;
    uint8_t temp = 0;
    do
    {
       ret = nrf_drv_twi_tx(&m_twi, BME280_7_ADDRESS, &reg, 1, true); //select register 
       if (NRF_SUCCESS != ret)
       {
           break;
       }
       ret = nrf_drv_twi_rx(&m_twi, BME280_7_ADDRESS, &temp, 1);
    }while (0);
    
    return temp;
    
}

 uint16_t BME280_read16(uint8_t reg)
{
   /* uint16_t temp ; 
    I2C_Master_Start();
    I2C_Master_Write(BME280_ADDRESS_W); //selection egistre
    I2C_Master_Wait();
    I2C_Master_Write(reg); //selection egistre
    I2C_Master_Wait();
    I2C_Master_Stop();
    I2C_Master_Start();
    I2C_Master_Write(BME280_ADDRESS_R); //lecture
    I2C_Master_Wait();
    temp = I2C_Master_Read(I2C_ACK);
    temp<<=8;
    temp |= I2C_Master_Read(I2C_NACK);
    I2C_Master_Wait();
    I2C_Master_Stop();*/
    ret_code_t ret;
    uint16_t temp = 0;
    uint8_t tab[2];
    do
    {
       ret = nrf_drv_twi_tx(&m_twi, BME280_7_ADDRESS, &reg, 1, true); //select register 
       if (NRF_SUCCESS != ret)
       {
           break;
       }
       ret = nrf_drv_twi_rx(&m_twi, BME280_7_ADDRESS, &tab, 2);
    }while (0);
    
    temp = tab[0];
    temp<<=8;
    temp |= tab[1];

    return temp;
    
}

inline uint32_t BME280_read24(uint8_t reg)
{
    /*uint32_t temp ; 
    I2C_Master_Start();
    I2C_Master_Write(BME280_ADDRESS_W); //selection egistre
    I2C_Master_Wait();
    I2C_Master_Write(reg); //selection egistre
    I2C_Master_Wait();
    I2C_Master_Stop();
    I2C_Master_Start();
    I2C_Master_Write(BME280_ADDRESS_R); //lecture
    I2C_Master_Wait();
    temp = I2C_Master_Read(I2C_ACK);
    temp<<=8;
    I2C_Master_Wait();
    temp |= I2C_Master_Read(I2C_ACK);
    temp<<=8;
    I2C_Master_Wait();
    temp |= I2C_Master_Read(I2C_NACK);
   // I2C_Master_Wait();
    I2C_Master_Stop();*/
    ret_code_t ret;
    uint32_t temp = 0;
    uint8_t tab[3];
    do
    {
       ret = nrf_drv_twi_tx(&m_twi, BME280_7_ADDRESS, &reg, 1, true); //select register 
       if (NRF_SUCCESS != ret)
       {
           break;
       }
       ret = nrf_drv_twi_rx(&m_twi, BME280_7_ADDRESS, &tab, 3);
    }while (0);
    temp = tab[0];
    temp<<=8;
    temp |=  tab[1];
    temp<<=8;
    temp |= tab[2];
    return temp;
    
}

int16_t BME280_readS16(uint8_t reg)
{
    return (int16_t)BME280_read16(reg);
}

 uint16_t BME280_read16_LE(uint8_t reg) {
    
    uint16_t temp =  BME280_read16(reg);
    
    return (temp >> 8) | (temp << 8);
    
}

int16_t BME280_readS16_LE(uint8_t reg)
{
    return (int16_t)BME280_read16_LE(reg);
}

 int8_t BME280_write8(uint8_t reg,uint8_t value)
{
    /* I2C_Master_Start();
    I2C_Master_Write(BME280_ADDRESS_W); //selection egistre
    I2C_Master_Wait();
    I2C_Master_Write(reg); //selection egistre
    I2C_Master_Wait();
    I2C_Master_Write(value); //selection egistre
    I2C_Master_Wait();
    I2C_Master_Stop(); */
        ret_code_t ret;
       uint8_t temp[2];
       temp[0] = reg;
       temp[1] = value;
       ret = nrf_drv_twi_tx(&m_twi, BME280_7_ADDRESS, &temp, 2, false); //select register 
       if (NRF_SUCCESS != ret)
       {
           return 1;
       }
       else
       {
            return 0;
       }
   
    
}


void InitBME280andPowerDown(void)
{
	BME280_write8(BME280_REGISTER_CONTROLHUMID, 0x01); // regler avant  CONTROL !!!
    BME280_write8(BME280_REGISTER_CONTROL, 0b00100100);
    readSensorCoefficients(); 
    displayCoefficients();
}

void Bme280_OneMeasure(int32_t *temp,int32_t * humi,float *press)
{
	 BME280_goForceMode();
     nrf_delay_ms(500);
     *temp = BME280_readTemperature();
     *humi = BME280_readHumidity();
     *press = BME280_readPressure();
}

void readSensorCoefficients(void)
{
    
    cal_data.dig_T1 = BME280_read16_LE(BME280_DIG_T1_REG);
    
    cal_data.dig_T2 = BME280_readS16_LE(BME280_DIG_T2_REG);
    
    cal_data.dig_T3 = BME280_readS16_LE(BME280_DIG_T3_REG);
    
    cal_data.dig_P1 = BME280_read16_LE(BME280_DIG_P1_REG);
    
    cal_data.dig_P2 = BME280_readS16_LE(BME280_DIG_P2_REG);
    
    cal_data.dig_P3 = BME280_readS16_LE(BME280_DIG_P3_REG);
    
    cal_data.dig_P4 = BME280_readS16_LE(BME280_DIG_P4_REG);
    
    cal_data.dig_P5 = BME280_readS16_LE(BME280_DIG_P5_REG);
    
    cal_data.dig_P6 = BME280_readS16_LE(BME280_DIG_P6_REG);
    
    cal_data.dig_P7 = BME280_readS16_LE(BME280_DIG_P7_REG);
    
    cal_data.dig_P8 = BME280_readS16_LE(BME280_DIG_P8_REG);
    
    cal_data.dig_P9 = BME280_readS16_LE(BME280_DIG_P9_REG);
    
    cal_data.dig_H1 = BME280_read8(BME280_DIG_H1_REG);
    
    cal_data.dig_H2 = BME280_readS16_LE(BME280_DIG_H2_REG);
    
    cal_data.dig_H3 = BME280_read8(BME280_DIG_H3_REG);
    
    cal_data.dig_H4 = (BME280_read8(BME280_DIG_H4_REG) << 4) | (BME280_read8(BME280_DIG_H4_REG+1) & 0xF);
    
    cal_data.dig_H5 = (BME280_read8(BME280_DIG_H5_REG+1) << 4) | (BME280_read8(BME280_DIG_H5_REG) >> 4);
    
    cal_data.dig_H6 = (int8_t)BME280_read8(BME280_DIG_H6_REG);


}

void displayCoefficients(void)
{
    NRF_LOG_INFO("dig_T1 : %u \r\n",cal_data.dig_T1);
    NRF_LOG_INFO("dig_T2 : %d \r\n",cal_data.dig_T2);
    NRF_LOG_INFO("dig_T3 : %d \r\n",cal_data.dig_T3);
    NRF_LOG_INFO("dig_P1 : %u \r\n",cal_data.dig_P1);
    NRF_LOG_INFO("dig_P2 : %d \r\n",cal_data.dig_P2);
    NRF_LOG_INFO("dig_P3 : %d \r\n",cal_data.dig_P3);
    NRF_LOG_INFO("dig_P4 : %d \r\n",cal_data.dig_P4);
    NRF_LOG_INFO("dig_P5 : %d \r\n",cal_data.dig_P5);
    NRF_LOG_INFO("dig_P6 : %d \r\n",cal_data.dig_P6);
    NRF_LOG_INFO("dig_P7 : %d \r\n",cal_data.dig_P7);
    NRF_LOG_INFO("dig_P8 : %d \r\n",cal_data.dig_P8);
    NRF_LOG_INFO("dig_P9 : %d \r\n",cal_data.dig_P9);
    NRF_LOG_INFO("dig_H1 : %u \r\n",cal_data.dig_H1);
    NRF_LOG_INFO("dig_H2 : %d \r\n",cal_data.dig_H2);
    NRF_LOG_INFO("dig_H3 : %u \r\n",cal_data.dig_H3);
    NRF_LOG_INFO("dig_H4 : %u \r\n",cal_data.dig_H4);
    NRF_LOG_INFO("dig_H5 : %u \r\n",cal_data.dig_H5);
    NRF_LOG_INFO("dig_H6 : %d \r\n",cal_data.dig_H6);
   
} 


void BME280_goForceMode(void)
{
    uint8_t mode ; 
    mode = BME280_read8(BME280_REGISTER_CONTROL);
    mode = (mode & 0xFC) + 0x01;
    BME280_write8(BME280_REGISTER_CONTROL,mode);
}

int32_t BME280_readTemperature(void)
{
    float temperature ;
    int32_t var1, var2;
    
    int32_t adc_T = BME280_read24(BME280_REGISTER_TEMPDATA);
        
    adc_T >>= 4;
    
    var1  = ((((adc_T>>3) - ((int32_t)cal_data.dig_T1 <<1))) *
             
             ((int32_t)cal_data.dig_T2)) >> 11;
    
    var2  = (((((adc_T>>4) - ((int32_t)cal_data.dig_T1)) *
               
               ((adc_T>>4) - ((int32_t)cal_data.dig_T1))) >> 12) *
             
             ((int32_t)cal_data.dig_T3)) >> 14;
    
    t_fine = var1 + var2;
  
    
    temperature  = (t_fine * 5 + 128) >> 8;
  
    
    
    return temperature ;
    
    
    
}




int32_t BME280_readHumidity(void) {
    
    int32_t adc_H = BME280_read16(BME280_REGISTER_HUMIDDATA);
    
    int32_t v_x1_u32r;
    
    v_x1_u32r = (t_fine - ((int32_t)76800));
    
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)cal_data.dig_H4) << 20) -
                    
                    (((int32_t)cal_data.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                 
                 (((((((v_x1_u32r * ((int32_t)cal_data.dig_H6)) >> 10) *
                      
                      (((v_x1_u32r * ((int32_t)cal_data.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                    
                    ((int32_t)2097152)) * ((int32_t)cal_data.dig_H2) + 8192) >> 14));
    
    
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                               
                               ((int32_t)cal_data.dig_H1)) >> 4));
    
    
 //   v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    
  //  v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
    
    int32_t h = (v_x1_u32r>>12);
    
    return  (h);
    
}
// Returns pressure in Pa as unsigned 32 bit integer. Output value of ?96386? equals 96386 Pa = 963.86 hPa
float BME280_readPressure(void)
{
     int64_t var1, var2, var3, var4;


  int32_t adc_P = BME280_read24(BME280_REGISTER_PRESSUREDATA);
  if (adc_P == 0x800000) // value in case pressure measurement was disabled
    return -1.0;
  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)cal_data.dig_P6;
  var2 = var2 + ((var1 * (int64_t)cal_data.dig_P5) * 131072);
  var2 = var2 + (((int64_t)cal_data.dig_P4) * 34359738368);
  var1 = ((var1 * var1 * (int64_t)cal_data.dig_P3) / 256) +
         ((var1 * ((int64_t)cal_data.dig_P2) * 4096));
  var3 = ((int64_t)1) * 140737488355328;
  var1 = (var3 + var1) * ((int64_t)cal_data.dig_P1) / 8589934592;

  if (var1 == 0) {
    return 0; // avoid exception caused by division by zero
  }

  var4 = 1048576 - adc_P;
  var4 = (((var4 * 2147483648) - var2) * 3125) / var1;
  var1 = (((int64_t)cal_data.dig_P9) * (var4 / 8192) * (var4 / 8192)) /
         33554432;
  var2 = (((int64_t)cal_data.dig_P8) * var4) / 524288;
  var4 = ((var4 + var1 + var2) / 256) + (((int64_t)cal_data.dig_P7) * 16);

  float P = var4 / 256.0;

  return P;
}




