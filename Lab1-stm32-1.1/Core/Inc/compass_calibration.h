#ifndef COMPASS_CALIBRATION_H
#define COMPASS_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  CAL_IDLE = 0, CAL_COLLECT = 1, CAL_NORTH = 2, CAL_TURN = 3, CAL_READY = 4
} CalibrationStage;

/* Stored in sensor counts and dimensionless units, before user percentages.
 * The positive symmetric 2x2 matrix corrects a horizontal field ellipse. */
typedef struct {
  float bias_x, bias_y;
  float m00, m01, m11;
  int32_t north;
  int32_t direction;
} CalibrationProfile;

void Calibration_Init(void);
void Calibration_Start(void);
void Calibration_Cancel(void);
void Calibration_Add(const int16_t raw[3]);
const char *Calibration_Finish(uint16_t sensitivity_nt);
const char *Calibration_SetNorth(void);
const char *Calibration_SetTurn(void);
CalibrationStage Calibration_Stage(void);
bool Calibration_Ready(void);
bool Calibration_Busy(void);
uint32_t Calibration_Samples(void);
uint32_t Calibration_Coverage(void);
bool Calibration_Load(const CalibrationProfile *profile);
const CalibrationProfile *Calibration_Profile(void);
void Calibration_Field(const int16_t raw[3], uint16_t sensitivity_nt,
                       const uint16_t gain[3], int32_t field[3]);
int32_t Calibration_Heading(const int32_t field[3]);

#endif
