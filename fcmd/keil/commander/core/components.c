#include "components.h"


/* ==============   COMPONENTS     CONFIGURATION CODE BEGIN    ============== */
/**
  * @brief  Set HTS221 temperature sensor Initialization.
  * @param  DeviceAddr: I2C device address
  * @param  InitStruct: pointer to a TSENSOR_InitTypeDef structure 
  *         that contains the configuration setting for the HTS221.
  */
void  HTS221_T_Init (uint16_t DeviceAddr)
{  
  uint8_t tmp[1];
  
  tmp[0] = i2c2_sensor_read(DeviceAddr, (uint8_t)0x10);
  tmp[0] &= ~(uint8_t)(0x3F);
  /* Apply settings to AV_CONF */
  i2c2_sensor_write(DeviceAddr, 0x10, tmp);
  
  
  /* Read CTRL_REG1 */
  tmp[0] = i2c2_sensor_read(DeviceAddr, (uint8_t)0x20);
  
  /* Enable BDU */
  tmp[0] &= ~(uint8_t)0x04;
  tmp[0] |= (1 <<  HTS221_BIT(2));
  
  /* Set default ODR */
  tmp[0] &= ~(uint8_t)0x03;
  tmp[0] |= (uint8_t)0x01; /* Set ODR to 1Hz */
  
  /* Activate the device */
  tmp[0] |= (uint8_t)0x80;
  
  
  /* Apply settings to CTRL_REG1 */
  i2c2_sensor_write(DeviceAddr, 0x20, tmp);
}

/**
  * @brief  Read temperature value of HTS221
  * @param  DeviceAddr: I2C device address
  * @retval temperature value
  */
float HTS221_T_ReadTemp (uint16_t DeviceAddr)
{
  int16_t T0_out, T1_out, T_out, T0_degC_x8_u16, T1_degC_x8_u16;
  int16_t T0_degC, T1_degC;
  uint8_t buffer[4], tmp;
  float tmp_f;

  i2c2_sensor_readmultiple(DeviceAddr, 0x32, buffer, 2);

  tmp = i2c2_sensor_read(DeviceAddr, 0x35);
  T0_degC_x8_u16 = (((uint16_t)(tmp & 0x03)) << 8) | ((uint16_t)buffer[0]);
  T1_degC_x8_u16 = (((uint16_t)(tmp & 0x0C)) << 6) | ((uint16_t)buffer[1]);
  T0_degC = T0_degC_x8_u16 >> 3;
  T1_degC = T1_degC_x8_u16 >> 3;

  i2c2_sensor_readmultiple(DeviceAddr, 0x3C, buffer, 4);

  T0_out = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];
  T1_out = (((uint16_t)buffer[3]) << 8) | (uint16_t)buffer[2];

  i2c2_sensor_readmultiple(DeviceAddr, 0x2A, buffer, 2);

  T_out = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];

  tmp_f = (float)(T_out - T0_out) * (float)(T1_degC - T0_degC) / (float)(T1_out - T0_out)  +  T0_degC;
/*  terminal("\nT0_degC: %d",T0_degC);
 terminal("\nT1_degC: %d",T1_degC);
 terminal("\n");
 terminal("\nT0_out: %d",T0_out);
 terminal("\nT1_out: %d",T1_out);
 terminal("\n");
 terminal("\nT_out:   %d",T_out);
 terminal("\n");
 */
	//terminal("\nTemperature:%f",tmp_f);
  return tmp_f;
}

/** @defgroup HTS221_Humidity_Private_Functions HTS221 Humidity Private Functions
  * @{
  */
/**
  * @brief  Set HTS221 humidity sensor Initialization.
  */
void  HTS221_H_Init  (uint16_t DeviceAddr)
{
  uint8_t tmp[0];
  
  /* Read CTRL_REG1 */
  tmp[0] = i2c2_sensor_read(DeviceAddr, 0x20);
  
  /* Enable BDU */
  tmp[0] &= ~0x04;
  tmp[0] |= (1 << HTS221_BIT(2));
  
  /* Set default ODR */
  tmp[0] &= ~0x03;
  tmp[0] |= (uint8_t)0x01; /* Set ODR to 1Hz */
  
  /* Activate the device */
  tmp[0] |= 0x80;
  
  /* Apply settings to CTRL_REG1 */
  i2c2_sensor_write(DeviceAddr, 0x20, tmp);
}

/**
  * @brief  Read humidity value of HTS221
  * @retval humidity value;
  */
float HTS221_H_ReadHumidity(uint16_t DeviceAddr)
{
  int16_t H0_T0_out, H1_T0_out, H_T_out;
  int16_t H0_rh, H1_rh;
  uint8_t buffer[2];
  float tmp_f;

  i2c2_sensor_readmultiple(DeviceAddr, 0x30, buffer, 2);

  H0_rh = buffer[0] >> 1;
  H1_rh = buffer[1] >> 1;

  i2c2_sensor_readmultiple(DeviceAddr, 0x36, buffer, 2);

  H0_T0_out = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];

  i2c2_sensor_readmultiple(DeviceAddr, 0x3A, buffer, 2);

  H1_T0_out = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];

  i2c2_sensor_readmultiple(DeviceAddr, 0x28, buffer, 2);

  H_T_out = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];

  tmp_f = (float)(H_T_out - H0_T0_out) * (float)(H1_rh - H0_rh) / (float)(H1_T0_out - H0_T0_out)  +  H0_rh;
  tmp_f *= 10.0f;

  tmp_f = ( tmp_f > 1000.0f ) ? 1000.0f
        : ( tmp_f <    0.0f ) ?    0.0f
        : tmp_f;
				
				
	tmp_f /= 10.0f;
	//terminal("\nHumdity:%f",tmp_f);
  return tmp_f;
}


