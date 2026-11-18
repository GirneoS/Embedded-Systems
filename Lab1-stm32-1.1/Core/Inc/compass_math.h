#ifndef COMPASS_MATH_H
#define COMPASS_MATH_H

#include <stdint.h>

/* Field units: 0.01 microtesla. Heading: clockwise from screen up, 0.01 degree;
 * -1 means the horizontal field is too small to determine a direction. */
void Compass_Scale(const int16_t raw[3], uint16_t sensitivity_nt,
                   const uint16_t gain_percent[3], int32_t field[3]);
int32_t Compass_Heading(const int32_t field[3]);

#endif
