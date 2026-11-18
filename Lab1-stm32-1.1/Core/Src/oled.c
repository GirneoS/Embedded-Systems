#include "oled.h"
#include "i2c.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define OLED_ADDRESS (0x3CU << 1)
#define IO_ADDRESS (0x70U << 1)
#define OLED_TIMEOUT_MS 30U

static uint8_t framebuffer[1024];
static unsigned next_page = 8U;
static bool online;

static bool IoRead(uint8_t reg, uint8_t *value)
{
  return HAL_I2C_Mem_Read(&hi2c1, IO_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT,
                          value, 1U, OLED_TIMEOUT_MS) == HAL_OK;
}

static bool IoWrite(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c1, IO_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT,
                           &value, 1U, OLED_TIMEOUT_MS) == HAL_OK;
}

bool Oled_Init(void)
{
  uint8_t output, config;
  /* SDK-1.1M manual, p.30: reset is P0 of PCA9538, address byte 0xE0.
   * Preserve the other seven outputs and their direction bits. */
  uint8_t commands[] = {
    0x00U, 0xAEU, 0x20U, 0x00U, 0xC8U, 0x40U, 0x81U, 0xCFU,
    0xA1U, 0xA6U, 0xA8U, 0x3FU, 0xA4U, 0xD3U, 0x00U,
    0xD5U, 0x80U, 0xD9U, 0x22U, 0xDAU, 0x12U, 0xDBU, 0x20U,
    0x8DU, 0x14U, 0xAFU
  };
  online = false;
  next_page = 8U;
  if (!IoRead(0x01U, &output) || !IoRead(0x03U, &config) ||
      !IoWrite(0x01U, output | 1U) || !IoWrite(0x03U, config & 0xFEU))
    return false;
  HAL_Delay(1U);
  if (!IoWrite(0x01U, output & 0xFEU)) return false;
  HAL_Delay(1U);
  if (!IoWrite(0x01U, output | 1U)) return false;
  HAL_Delay(1U);
  online = HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDRESS, commands,
                                  sizeof(commands), OLED_TIMEOUT_MS) == HAL_OK;
  return online;
}

bool Oled_Online(void) { return online; }
bool Oled_Busy(void) { return next_page < 8U; }

static void Pixel(int x, int y)
{
  if (x >= 0 && x < 128 && y >= 0 && y < 64)
    framebuffer[(unsigned)(y / 8) * 128U + (unsigned)x] |= (uint8_t)(1U << (y % 8));
}

static void Line(int x0, int y0, int x1, int y1)
{
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int error = dx + dy;
  for (;;)
  {
    int twice;
    Pixel(x0, y0);
    if (x0 == x1 && y0 == y1) break;
    twice = 2 * error;
    if (twice >= dy) { error += dy; x0 += sx; }
    if (twice <= dx) { error += dx; y0 += sy; }
  }
}

/* Original minimal 5x7 bitmap alphabet; columns, least-significant bit on top. */
static const uint8_t *Glyph(char c)
{
  static const char alphabet[] = "0123456789XYZBUTHNAOSERWI:+-.?CL";
  static const uint8_t glyphs[][5] = {
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
    {0x62,0x51,0x49,0x49,0x46}, {0x22,0x49,0x49,0x49,0x36},
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
    {0x3E,0x49,0x49,0x49,0x32}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x26,0x49,0x49,0x49,0x3E},
    {0x63,0x14,0x08,0x14,0x63}, {0x03,0x04,0x78,0x04,0x03},
    {0x61,0x51,0x49,0x45,0x43}, {0x7F,0x49,0x49,0x49,0x36},
    {0x3F,0x40,0x40,0x40,0x3F}, {0x01,0x01,0x7F,0x01,0x01},
    {0x7F,0x08,0x08,0x08,0x7F}, {0x7F,0x02,0x04,0x08,0x7F},
    {0x7E,0x09,0x09,0x09,0x7E}, {0x3E,0x41,0x41,0x41,0x3E},
    {0x26,0x49,0x49,0x49,0x32}, {0x7F,0x49,0x49,0x49,0x41},
    {0x7F,0x09,0x19,0x29,0x46}, {0x3F,0x40,0x38,0x40,0x3F},
    {0x00,0x41,0x7F,0x41,0x00}, {0x00,0x36,0x36,0x00,0x00},
    {0x08,0x08,0x3E,0x08,0x08}, {0x08,0x08,0x08,0x08,0x08},
    {0x00,0x60,0x60,0x00,0x00}, {0x02,0x01,0x51,0x09,0x06},
    {0x3E,0x41,0x41,0x41,0x22}, {0x7F,0x40,0x40,0x40,0x40}
  };
  static const uint8_t blank[5] = {0};
  const char *found = strchr(alphabet, c);
  return found != NULL && c != '\0' ? glyphs[found - alphabet] : blank;
}

static void Text(int x, int y, const char *text)
{
  while (*text != '\0' && x <= 122)
  {
    const uint8_t *glyph = Glyph(*text++);
    unsigned col, row;
    for (col = 0U; col < 5U; ++col)
      for (row = 0U; row < 7U; ++row)
        if ((glyph[col] & (1U << row)) != 0U) Pixel(x + (int)col, y + (int)row);
    x += 6;
  }
}

void Oled_Render(const CompassSample *sample)
{
  char text[24];
  unsigned i;
  if (!online || Oled_Busy()) return;
  memset(framebuffer, 0, sizeof(framebuffer));
  Text(0, 0, "B UT");
  for (i = 0U; i < 3U; ++i)
  {
    if ((sample->flags & COMPASS_FLAG_SAMPLE) != 0U)
    {
      int32_t value = sample->field[i];
      uint32_t magnitude = value < 0 ? (uint32_t)-value : (uint32_t)value;
      (void)snprintf(text, sizeof(text), "%c:%c%lu.%02lu", "XYZ"[i],
                     value < 0 ? '-' : '+', (unsigned long)(magnitude / 100U),
                     (unsigned long)(magnitude % 100U));
    }
    else
      (void)snprintf(text, sizeof(text), "%c:---", "XYZ"[i]);
    Text(0, 12 + (int)i * 12, text);
  }
  if ((sample->flags & COMPASS_FLAG_HEADING) != 0U)
  {
    float angle = (float)sample->heading * (3.14159265358979323846f / 18000.0f);
    float dx = sinf(angle), dy = -cosf(angle);
    int tip_x = 105 + (int)lroundf(19.0f * dx);
    int tip_y = 31 + (int)lroundf(19.0f * dy);
    Line(105, 31, tip_x, tip_y);
    Line(tip_x, tip_y, tip_x + (int)lroundf(-7.0f * dx + 4.0f * dy),
         tip_y + (int)lroundf(-7.0f * dy - 4.0f * dx));
    Line(tip_x, tip_y, tip_x + (int)lroundf(-7.0f * dx - 4.0f * dy),
         tip_y + (int)lroundf(-7.0f * dy + 4.0f * dx));
    (void)snprintf(text, sizeof(text), "H:%lu.%02lu",
                   (unsigned long)sample->heading / 100U,
                   (unsigned long)sample->heading % 100U);
    Text(0, 48, text);
  }
  else
  {
    Text(0, 48, "H:---");
    Text(102, 28, "?");
  }
  Text(102, 0, "0"); /* Screen-up reference, not a fixed north label. */
  for (i = 0U; i < 360U; i += 12U)
  {
    float a = (float)i * (3.14159265358979323846f / 180.0f);
    Pixel(105 + (int)lroundf(21.0f * cosf(a)), 31 + (int)lroundf(21.0f * sinf(a)));
  }
  Text(0, 57, (sample->flags & COMPASS_FLAG_SAMPLE) == 0U ? "NO SENSOR" :
               (sample->flags & COMPASS_FLAG_SATURATED) != 0U ? "SATURATION" :
               (sample->flags & COMPASS_FLAG_CALIBRATING) != 0U ? "CAL..." :
               (sample->flags & COMPASS_FLAG_CALIBRATED) == 0U ? "CAL REQUIRED" :
               (sample->flags & COMPASS_FLAG_HEADING) == 0U ? "NO XY" : "");
  next_page = 0U;
}

void Oled_Process(void)
{
  uint8_t commands[7], data[129];
  if (!online || !Oled_Busy()) return;
  /* Limit each transaction to one page: about 12 ms at 100 kHz. UART and
   * magnetometer service run again before the following page is sent. */
  commands[0] = 0x00U;
  commands[1] = 0x21U; commands[2] = 0U; commands[3] = 127U;
  commands[4] = 0x22U; commands[5] = (uint8_t)next_page;
  commands[6] = (uint8_t)next_page;
  data[0] = 0x40U;
  memcpy(data + 1, framebuffer + next_page * 128U, 128U);
  if (HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDRESS, commands, sizeof(commands),
                              OLED_TIMEOUT_MS) != HAL_OK ||
      HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDRESS, data, sizeof(data),
                              OLED_TIMEOUT_MS) != HAL_OK)
  {
    online = false;
    next_page = 8U;
    return;
  }
  ++next_page;
}
