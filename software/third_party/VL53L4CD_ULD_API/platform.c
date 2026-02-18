/*
 * This file includes code provided by STMicroelectronics,
 * originally licensed under the BSD License (https://www.st.com/SLA0103).
 *
 * Modifications made by INESC TEC on communication functions:
 * - RdByte, RdWord, RdDWord, WrByte, WrWord, WrDWord, WaitMs
 */


#include "platform.h"

#define DEFAULT_I2C_BUFFER_LEN  32

i2c_inst_t* vl_i2c_inst_ = NULL;

uint8_t VL53L4CD_RdByte(Dev_t dev, uint16_t RegisterAdress, uint8_t *value)
{
  uint8_t status = 0;
  status = VL53L4CD_I2CRead(dev, RegisterAdress, value, 1);
  return status;
}

uint8_t VL53L4CD_RdWord(Dev_t dev, uint16_t RegisterAdress, uint16_t *value)
{
  uint8_t status = 0;
  uint8_t buffer[2] = {0, 0};

  status = VL53L4CD_I2CRead(dev, RegisterAdress, buffer, 2);
  if (!status)
    *value = ((uint16_t) buffer[0] << 8) + (uint16_t) buffer[1];

  return status;
}

uint8_t VL53L4CD_RdDWord(Dev_t dev, uint16_t RegisterAdress, uint32_t *value)
{
  uint8_t status = 0;
  uint8_t buffer[4] = {0, 0, 0, 0};

  status = VL53L4CD_I2CRead(dev, RegisterAdress, buffer, 4);
  if (!status)
    *value = ((uint32_t) buffer[0] << 24) + ((uint32_t) buffer[1] << 16) + ((uint32_t) buffer[2] << 8) + (uint32_t) buffer[3];

  return status;
}

uint8_t VL53L4CD_WrByte(Dev_t dev, uint16_t RegisterAdress, uint8_t value)
{
  uint8_t status = 0;
  status = VL53L4CD_I2CWrite(dev, RegisterAdress, &value, 1);
  return status;
}

uint8_t VL53L4CD_WrWord(Dev_t dev, uint16_t RegisterAdress, uint16_t value)
{
  uint8_t status = 0;
  uint8_t buffer[2];

  buffer[0] = value >> 8;
  buffer[1] = value & 0xFF;
  status = VL53L4CD_I2CWrite(dev, RegisterAdress, (uint8_t*) buffer, 2);
  return status;
}

uint8_t VL53L4CD_WrDWord(Dev_t dev, uint16_t RegisterAdress, uint32_t value)
{
  uint8_t status = 0;
  uint8_t buffer[4];

  buffer[0] = (value >> 24) & 0xFF;
  buffer[1] = (value >> 16) & 0xFF;
  buffer[2] = (value >>  8) & 0xFF;
  buffer[3] = (value >>  0) & 0xFF;
  status = VL53L4CD_I2CWrite(dev, RegisterAdress, (uint8_t*) buffer, 4);
  return status;
}

uint8_t VL53L4CD_WaitMs(Dev_t dev, uint32_t TimeMs)
{
  uint8_t status = 255;
  sleep_ms(TimeMs);
  status = 0;

  return status;
}

uint8_t VL53L4CD_I2CRead(uint8_t DeviceAddr, uint16_t RegisterAddress, uint8_t *values, uint32_t size)
{
  uint8_t status = 0;
  uint8_t buffer[DEFAULT_I2C_BUFFER_LEN];
  uint8_t dev_addr_7bits = (uint8_t)(DeviceAddr >> 1) & 0x7F;

  buffer[0] = (uint8_t)((RegisterAddress >> 8) & 0xFF);
  buffer[1] = (uint8_t)(RegisterAddress & 0xFF);

  absolute_time_t timeout_time = make_timeout_time_ms(1000);
  int stat = i2c_write_blocking_until(vl_i2c_inst_, dev_addr_7bits, buffer, 2, false, timeout_time);
  if (stat == PICO_ERROR_GENERIC)
  {
    printf("Error I2CRead(): Generic error on write.\n");
    status = 255;
  }
  else if (stat == PICO_ERROR_TIMEOUT)
  {
    printf("Error I2CRead(): Timeout error on write.\n");
    status = 255;
  }
  else if (stat != 2)
  {
    printf("Error I2CRead(): Written bytes mismatch on write. Written %d bytes.\n", stat);
    status = 255;
  }

  timeout_time = make_timeout_time_ms(1000);
  stat = i2c_read_blocking_until(vl_i2c_inst_, dev_addr_7bits, values, size, false, timeout_time);
  if (stat == PICO_ERROR_GENERIC)
  {
    printf("Error I2CRead(): Generic error on read.\n");
    status = 255;
  }
  else if (stat == PICO_ERROR_TIMEOUT)
  {
    printf("Error I2CRead(): Timeout error on read.\n");
    status = 255;
  }
  else if (stat != (int)size)
  {
    printf("Error I2CRead(): Read bytes mismatch. Read %d bytes.\n", stat);
    status = 255;
  }

  return status;
}

uint8_t VL53L4CD_I2CWrite(uint8_t DeviceAddr, uint16_t RegisterAddress, uint8_t *values, uint32_t size)
{
  uint8_t status = 0;
  uint8_t buffer[2];
  uint8_t dev_addr_7bits = (uint8_t)(DeviceAddr >> 1) & 0x7F;

  buffer[0] = (uint8_t)((RegisterAddress >> 8) & 0xFF);
  buffer[1] = (uint8_t)(RegisterAddress & 0xFF);

  for (uint32_t i = 0; i < size; ++i)
  {
    buffer[i+2] = values[i];
  }

  absolute_time_t timeout_time = make_timeout_time_ms(1000);
  int stat = i2c_write_blocking_until(vl_i2c_inst_, dev_addr_7bits, buffer, size + 2, false, timeout_time);

  if (stat == PICO_ERROR_GENERIC)
  {
    printf("Error I2CWrite(): Generic error.\n");
    status = 255;
  }
  else if (stat == PICO_ERROR_TIMEOUT)
  {
    printf("Error I2CWrite(): Timeout error.\n");
    status = 255;
  }
  else if (stat != (int)(size + 2))
  {
    printf("Error I2CWrite(): Written bytes mismatch. Written %d bytes.\n", stat);
    status = 255;
  }

  return status;
}
