#include "magnetometer.h"
#include "i2c.h"
#include <stddef.h>

#define MAG_TIMEOUT_MS 20U

static bool Read(uint8_t address, uint8_t reg, uint8_t *data, uint16_t count)
{
  return HAL_I2C_Mem_Read(&hi2c1, (uint16_t)address << 1, reg,
                          I2C_MEMADD_SIZE_8BIT, data, count, MAG_TIMEOUT_MS) == HAL_OK;
}

static bool Write(uint8_t address, uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c1, (uint16_t)address << 1, reg,
                           I2C_MEMADD_SIZE_8BIT, &value, 1U, MAG_TIMEOUT_MS) == HAL_OK;
}

static bool ClearOffsets(uint8_t address, uint8_t start)
{
  unsigned i;
  for (i = 0U; i < 6U; ++i)
    if (!Write(address, (uint8_t)(start + i), 0U)) return false;
  return true;
}

static bool ResetDS1(uint8_t address)
{
  uint8_t control;
  uint32_t started = HAL_GetTick();
  if (!Write(address, 0x21U, 0x04U)) return false; /* SOFT_RST */
  do
  {
    if (!Read(address, 0x21U, &control, 1U)) return false;
    if ((control & 0x04U) == 0U) return true;
    HAL_Delay(1U);
  } while ((uint32_t)(HAL_GetTick() - started) < 100U);
  return false;
}

bool Mag_Init(Magnetometer *sensor)
{
  static const uint8_t addresses[] = {0x1DU, 0x1EU, 0x1CU};
  unsigned i;
  sensor->who_am_i = 0U;
  sensor->address = 0U;
  sensor->sensitivity_nt = 0U;
  for (i = 0U; i < sizeof(addresses); ++i)
  {
    uint8_t id = 0U;
    bool ok = false;
    uint8_t address = addresses[i];
    if (!Read(address, 0x0FU, &id, 1U)) continue;
    if (id == 0x49U && address != 0x1CU)
    {
      /* LSM9DS0 XM: BDU, accelerometer powered down; magnetic high
       * resolution, 50 Hz, +/-12 gauss, continuous conversion.
       * ST DocID024763 tables 3, 71, 82-91. */
      /* Explicitly undo magnetic offset/self-test settings left by another
       * program. DS0 has no DS1-style SOFT_RST register. */
      ok = Write(address, 0x26U, 0x03U) &&
           Write(address, 0x1FU, 0x00U) &&
           ClearOffsets(address, 0x16U) &&
           Write(address, 0x20U, 0x08U) &&
           Write(address, 0x24U, 0x70U) &&
           Write(address, 0x25U, 0x60U) &&
           Write(address, 0x26U, 0x00U);
      sensor->sensitivity_nt = 48U;
    }
    else if (id == 0x3DU && address != 0x1DU)
    {
      /* LSM9DS1 M: temperature compensation, ultra-high performance XY/Z,
       * 40 Hz, +/-16 gauss, BDU, continuous conversion.
       * ST DocID025715 tables 3, 108-122. */
      ok = ResetDS1(address) &&
           ClearOffsets(address, 0x05U) &&
           Write(address, 0x21U, 0x60U) &&
           Write(address, 0x20U, 0xF8U) &&
           Write(address, 0x23U, 0x0CU) &&
           Write(address, 0x24U, 0x40U) &&
           Write(address, 0x22U, 0x00U);
      sensor->sensitivity_nt = 58U;
    }
    if (ok)
    {
      sensor->address = address;
      sensor->who_am_i = id;
      return true;
    }
  }
  sensor->sensitivity_nt = 0U;
  return false;
}

const char *Mag_Name(const Magnetometer *sensor)
{
  if (sensor->who_am_i == 0x49U) return "LSM9DS0";
  if (sensor->who_am_i == 0x3DU) return "LSM9DS1";
  return "NONE";
}

MagResult Mag_Read(const Magnetometer *sensor, int16_t raw[3])
{
  uint8_t status, bytes[6];
  uint8_t status_reg = sensor->who_am_i == 0x49U ? 0x07U : 0x27U;
  uint8_t data_reg = sensor->who_am_i == 0x49U ? 0x08U : 0x28U;
  unsigned i;
  if (sensor->who_am_i == 0U || !Read(sensor->address, status_reg, &status, 1U))
    return MAG_ERROR;
  if ((status & 0x08U) == 0U) return MAG_WAIT;
  /* Subaddress bit 7 enables address auto-increment for a six-byte read. */
  if (!Read(sensor->address, data_reg | 0x80U, bytes, 6U)) return MAG_ERROR;
  for (i = 0U; i < 3U; ++i)
  {
    uint16_t word = (uint16_t)bytes[2U * i] | ((uint16_t)bytes[2U * i + 1U] << 8);
    raw[i] = word <= 32767U ? (int16_t)word : (int16_t)((int32_t)word - 65536);
  }
  return MAG_READY;
}
