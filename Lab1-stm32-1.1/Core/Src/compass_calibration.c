#include "compass_calibration.h"
#include "compass_math.h"
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define POINT_LIMIT 1024U
#define RECENT_LIMIT 12U
#define PI_F 3.14159265358979323846f

static int16_t points[POINT_LIMIT][2], recent[RECENT_LIMIT][2];
static uint32_t count, total, stride, recent_count, recent_head, coverage;
static CalibrationProfile active, candidate;
static CalibrationStage stage;
static bool ready;

static void Identity(CalibrationProfile *p)
{
  memset(p, 0, sizeof(*p));
  p->m00 = p->m11 = 1.0f;
  p->direction = 1;
}

void Calibration_Init(void)
{
  Identity(&active);
  Identity(&candidate);
  ready = false;
  stage = CAL_IDLE;
  count = total = recent_count = recent_head = coverage = 0U;
  stride = 1U;
}

CalibrationStage Calibration_Stage(void) { return stage; }
bool Calibration_Ready(void) { return ready; }
bool Calibration_Busy(void) { return stage >= CAL_COLLECT && stage <= CAL_TURN; }
uint32_t Calibration_Samples(void) { return total; }
uint32_t Calibration_Coverage(void) { return coverage; }
const CalibrationProfile *Calibration_Profile(void) { return &active; }

void Calibration_Start(void)
{
  /* START retries must not erase an acquisition already in progress. */
  if (Calibration_Busy()) return;
  Identity(&candidate);
  count = total = recent_count = recent_head = coverage = 0U;
  stride = 1U;
  stage = CAL_COLLECT;
}

void Calibration_Cancel(void)
{
  stage = ready ? CAL_READY : CAL_IDLE;
}

void Calibration_Add(const int16_t raw[3])
{
  uint32_t i;
  recent[recent_head][0] = raw[0];
  recent[recent_head][1] = raw[1];
  recent_head = (recent_head + 1U) % RECENT_LIMIT;
  if (recent_count < RECENT_LIMIT) ++recent_count;
  if (stage != CAL_COLLECT) return;
  ++total;
  if (total % stride != 0U) return;
  if (count == POINT_LIMIT)
  {
    /* Uniformly decimate instead of forgetting the beginning of a turn. */
    for (i = 0U; i < POINT_LIMIT / 2U; ++i)
    {
      points[i][0] = points[2U * i][0];
      points[i][1] = points[2U * i][1];
    }
    count = POINT_LIMIT / 2U;
    stride *= 2U;
  }
  points[count][0] = raw[0];
  points[count][1] = raw[1];
  ++count;
}

static bool Solve(float a[5][6], float result[5])
{
  unsigned col, row, j;
  for (col = 0U; col < 5U; ++col)
  {
    unsigned pivot = col;
    for (row = col + 1U; row < 5U; ++row)
      if (fabsf(a[row][col]) > fabsf(a[pivot][col])) pivot = row;
    if (fabsf(a[pivot][col]) < 0.00001f) return false;
    for (j = col; j < 6U; ++j)
    {
      float temporary = a[col][j];
      a[col][j] = a[pivot][j];
      a[pivot][j] = temporary;
    }
    {
      float divisor = a[col][col];
      for (j = col; j < 6U; ++j) a[col][j] /= divisor;
    }
    for (row = 0U; row < 5U; ++row)
      if (row != col)
      {
        float factor = a[row][col];
        for (j = col; j < 6U; ++j) a[row][j] -= factor * a[col][j];
      }
  }
  for (row = 0U; row < 5U; ++row) result[row] = a[row][5];
  return true;
}

static void Correct(const CalibrationProfile *p, float x, float y, float *cx, float *cy)
{
  x -= p->bias_x;
  y -= p->bias_y;
  *cx = p->m00 * x + p->m01 * y;
  *cy = p->m01 * x + p->m11 * y;
}

static bool Valid(const CalibrationProfile *p)
{
  float determinant = p->m00 * p->m11 - p->m01 * p->m01;
  return isfinite(p->bias_x) && isfinite(p->bias_y) &&
         fabsf(p->bias_x) <= 32768.0f && fabsf(p->bias_y) <= 32768.0f &&
         isfinite(p->m00) && isfinite(p->m01) && isfinite(p->m11) &&
         p->m00 >= 0.2f && p->m00 <= 5.0f && p->m11 >= 0.2f && p->m11 <= 5.0f &&
         determinant >= 0.8f && determinant <= 1.2f &&
         p->north >= 0 && p->north < 36000 &&
         (p->direction == 1 || p->direction == -1);
}

const char *Calibration_Finish(uint16_t sensitivity_nt)
{
  float min_x = 32768.0f, min_y = 32768.0f;
  float max_x = -32768.0f, max_y = -32768.0f;
  float mid_x, mid_y, half_x, half_y, normal[5][6] = {{0}}, fit[5];
  float determinant, cx, cy, k, q00, q01, q11, root_det, denominator, radius;
  float squared_error = 0.0f;
  uint32_t i, bins = 0U;
  unsigned j, l;
  CalibrationProfile p;
  if (stage == CAL_NORTH || stage == CAL_TURN) return NULL;
  if (stage != CAL_COLLECT) return "CAL_STATE";
  if (count < 100U) return "CAL_SAMPLES";
  for (i = 0U; i < count; ++i)
  {
    if (points[i][0] < min_x) min_x = points[i][0];
    if (points[i][0] > max_x) max_x = points[i][0];
    if (points[i][1] < min_y) min_y = points[i][1];
    if (points[i][1] > max_y) max_y = points[i][1];
  }
  half_x = (max_x - min_x) * 0.5f;
  half_y = (max_y - min_y) * 0.5f;
  if (half_x * sensitivity_nt < 2000.0f || half_y * sensitivity_nt < 2000.0f)
    return "CAL_RANGE";
  mid_x = (max_x + min_x) * 0.5f;
  mid_y = (max_y + min_y) * 0.5f;
  /* Least-squares conic in normalized coordinates:
   * A*x*x + B*x*y + C*y*y + D*x + E*y = 1. */
  for (i = 0U; i < count; ++i)
  {
    float x = (points[i][0] - mid_x) / half_x;
    float y = (points[i][1] - mid_y) / half_y;
    float v[5] = {x*x, x*y, y*y, x, y};
    for (j = 0U; j < 5U; ++j)
    {
      normal[j][5] += v[j];
      for (l = 0U; l < 5U; ++l) normal[j][l] += v[j] * v[l];
    }
  }
  if (!Solve(normal, fit)) return "CAL_COVERAGE";
  determinant = fit[0] * fit[2] - 0.25f * fit[1] * fit[1];
  if (fit[0] <= 0.0f || fit[2] <= 0.0f || determinant <= 0.000001f)
    return "CAL_DISTORTION";
  cx = -(fit[2] * fit[3] - 0.5f * fit[1] * fit[4]) / (2.0f * determinant);
  cy = -(fit[0] * fit[4] - 0.5f * fit[1] * fit[3]) / (2.0f * determinant);
  k = 1.0f + fit[0]*cx*cx + fit[1]*cx*cy + fit[2]*cy*cy;
  if (k <= 0.0f) return "CAL_DISTORTION";
  q00 = fit[0] / (k * half_x * half_x);
  q01 = fit[1] / (2.0f * k * half_x * half_y);
  q11 = fit[2] / (k * half_y * half_y);
  determinant = q00 * q11 - q01 * q01;
  if (determinant <= 0.0f) return "CAL_DISTORTION";
  root_det = sqrtf(determinant);
  denominator = sqrtf(q00 + q11 + 2.0f * root_det);
  radius = 1.0f / sqrtf(root_det);
  Identity(&p);
  p.bias_x = mid_x + half_x * cx;
  p.bias_y = mid_y + half_y * cy;
  /* Positive square root of the ellipse matrix, preserving its area.
   * Unlike separate min/max gains, this also corrects an inclined ellipse. */
  p.m00 = radius * (q00 + root_det) / denominator;
  p.m01 = radius * q01 / denominator;
  p.m11 = radius * (q11 + root_det) / denominator;
  if (!Valid(&p)) return "CAL_DISTORTION";
  for (i = 0U; i < count; ++i)
  {
    float x, y, error, angle;
    unsigned bin;
    Correct(&p, points[i][0], points[i][1], &x, &y);
    error = sqrtf(x*x + y*y) / radius - 1.0f;
    squared_error += error * error;
    if (fabsf(error) > 0.35f) return "CAL_DISTORTION";
    angle = atan2f(y, x) + PI_F;
    bin = (unsigned)(angle * (8.0f / PI_F)) % 16U;
    bins |= 1UL << bin;
  }
  coverage = 0U;
  for (i = 0U; i < 16U; ++i) if ((bins & (1UL << i)) != 0U) ++coverage;
  if (coverage < 14U) return "CAL_COVERAGE";
  if (sqrtf(squared_error / count) > 0.10f) return "CAL_DISTORTION";
  candidate = p;
  recent_count = recent_head = 0U;
  stage = CAL_NORTH;
  return NULL;
}

static int32_t Wrap(int32_t angle)
{
  angle %= 36000;
  return angle < 0 ? angle + 36000 : angle;
}

static const char *Reference(int32_t *angle)
{
  float x = 0.0f, y = 0.0f, mean_x, mean_y;
  uint32_t i;
  if (recent_count < RECENT_LIMIT) return "CAL_STILL";
  for (i = 0U; i < RECENT_LIMIT; ++i)
  {
    float cx, cy;
    Correct(&candidate, recent[i][0], recent[i][1], &cx, &cy);
    x += cx;
    y += cy;
  }
  mean_x = x / RECENT_LIMIT;
  mean_y = y / RECENT_LIMIT;
  if (mean_x*mean_x + mean_y*mean_y < 25.0f) return "CAL_RANGE";
  *angle = Wrap((int32_t)lroundf(atan2f(mean_x, mean_y) * (18000.0f / PI_F)));
  for (i = 0U; i < RECENT_LIMIT; ++i)
  {
    int32_t delta;
    Correct(&candidate, recent[i][0], recent[i][1], &x, &y);
    delta = Wrap((int32_t)lroundf(atan2f(x, y) * (18000.0f / PI_F)) - *angle + 18000) - 18000;
    if (delta < -500 || delta > 500) return "CAL_STILL";
  }
  return NULL;
}

const char *Calibration_SetNorth(void)
{
  const char *error;
  if (stage == CAL_TURN) return NULL;
  if (stage != CAL_NORTH) return "CAL_STATE";
  error = Reference(&candidate.north);
  if (error != NULL) return error;
  recent_count = recent_head = 0U;
  stage = CAL_TURN;
  return NULL;
}

const char *Calibration_SetTurn(void)
{
  int32_t angle, delta;
  const char *error;
  if (stage == CAL_READY) return NULL;
  if (stage != CAL_TURN) return "CAL_STATE";
  error = Reference(&angle);
  if (error != NULL) return error;
  delta = Wrap(angle - candidate.north + 18000) - 18000;
  if (abs(delta) < 6000 || abs(delta) > 12000) return "CAL_TURN90";
  candidate.direction = delta > 0 ? -1 : 1;
  active = candidate;
  ready = true;
  stage = CAL_READY;
  return NULL;
}

bool Calibration_Load(const CalibrationProfile *profile)
{
  if (!Valid(profile) || Calibration_Busy()) return false;
  active = *profile;
  ready = true;
  stage = CAL_READY;
  return true;
}

void Calibration_Field(const int16_t raw[3], uint16_t sensitivity_nt,
                       const uint16_t gain[3], int32_t field[3])
{
  float x, y;
  Compass_Scale(raw, sensitivity_nt, gain, field);
  if (!ready) return;
  Correct(&active, raw[0], raw[1], &x, &y);
  field[0] = (int32_t)lroundf(x * sensitivity_nt * gain[0] / 1000.0f);
  field[1] = (int32_t)lroundf(y * sensitivity_nt * gain[1] / 1000.0f);
}

int32_t Calibration_Heading(const int32_t field[3])
{
  int32_t angle;
  if (!ready || Calibration_Busy()) return -1;
  angle = Compass_Heading(field);
  return angle < 0 ? -1 : Wrap(active.direction * (angle - active.north));
}
