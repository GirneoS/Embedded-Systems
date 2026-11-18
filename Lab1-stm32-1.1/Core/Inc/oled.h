#ifndef OLED_H
#define OLED_H

#include "compass.h"
#include <stdbool.h>

bool Oled_Init(void);
bool Oled_Online(void);
bool Oled_Busy(void);
void Oled_Render(const CompassSample *sample);
/* Transfers at most one 128-byte page, always from the main loop. */
void Oled_Process(void);

#endif
