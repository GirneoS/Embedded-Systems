#ifndef COMPASS_H
#define COMPASS_H

#include <stdint.h>

#define COMPASS_FLAG_SAMPLE 1U
#define COMPASS_FLAG_HEADING 2U
#define COMPASS_FLAG_SATURATED 4U
#define COMPASS_FLAG_OLED 8U
#define COMPASS_FLAG_CALIBRATED 16U
#define COMPASS_FLAG_CALIBRATING 32U

typedef struct
{
  uint32_t tick_ms;
  int32_t field[3];
  int32_t heading;
  uint16_t gain[3];
  uint8_t flags;
} CompassSample;

void Compass_Init(void);
void Compass_Process(void);

#endif
