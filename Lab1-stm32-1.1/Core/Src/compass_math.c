#include "compass_math.h"
#include "compass_config.h"
#include <math.h>

void Compass_Scale(const int16_t raw[3], uint16_t sensitivity_nt,
                   const uint16_t gain_percent[3], int32_t field[3])
{
  unsigned i;
  for (i = 0U; i < 3U; ++i)
  {
    /* raw * nT/LSB * percent / 100 / 10 => hundredths of microtesla.
     * At 58 nT/LSB and 200% the intermediate fits in signed 32 bits. */
    int32_t value = (int32_t)raw[i] * sensitivity_nt * gain_percent[i];
    field[i] = value >= 0 ? (value + 500) / 1000 : (value - 500) / 1000;
  }
}

int32_t Compass_Heading(const int32_t field[3])
{
  float right = (float)field[0];
  float up = (float)field[1];
  float angle;
  if (right * right + up * up <
      COMPASS_MIN_HORIZONTAL_CUT * COMPASS_MIN_HORIZONTAL_CUT)
  {
    return -1;
  }
  angle = atan2f(right, up) * (18000.0f / 3.14159265358979323846f);
  if (angle < 0.0f)
  {
    angle += 36000.0f;
  }
  return (int32_t)(angle + 0.5f) % 36000;
}
