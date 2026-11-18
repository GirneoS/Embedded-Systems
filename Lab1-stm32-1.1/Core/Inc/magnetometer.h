#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint8_t address; /* Seven-bit I2C address, shifted only at the HAL boundary. */
  uint8_t who_am_i;
  uint16_t sensitivity_nt;
} Magnetometer;

typedef enum { MAG_ERROR = -1, MAG_WAIT = 0, MAG_READY = 1 } MagResult;

bool Mag_Init(Magnetometer *sensor);
const char *Mag_Name(const Magnetometer *sensor);
MagResult Mag_Read(const Magnetometer *sensor, int16_t raw[3]);

#endif
