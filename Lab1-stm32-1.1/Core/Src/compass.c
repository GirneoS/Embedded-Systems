#include "compass.h"
#include "compass_config.h"
#include "compass_calibration.h"
#include "compass_math.h"
#include "magnetometer.h"
#include "oled.h"
#include "usart.h"
#include "wire_protocol.h"

#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define RX_CAPACITY 512U

static volatile uint8_t rx_buffer[RX_CAPACITY];
static volatile uint16_t rx_head, rx_tail;
static volatile bool rx_overflow, rx_restart;
static uint8_t irq_byte;
static char rx_frame[WIRE_FRAME_SIZE];
static size_t rx_length;
static bool collecting;

static Magnetometer sensor;
static bool sensor_online, sample_valid, saturated;
static int16_t raw[3];
static uint16_t gain[3] = {100U, 100U, 100U};
static uint32_t last_sample, last_poll, last_frame, last_sensor_retry, last_oled_retry;
static uint32_t calibration_activity;

static void Send(const char *body)
{
  char frame[WIRE_FRAME_SIZE];
  size_t length = Wire_Encode(body, frame, sizeof(frame));
  if (length != 0U)
    (void)HAL_UART_Transmit(&huart6, (const uint8_t *)frame, (uint16_t)length, 100U);
}

static void ReplyError(uint32_t sequence, const char *reason)
{
  char body[64];
  (void)snprintf(body, sizeof(body), "%lu,ERR,%s", (unsigned long)sequence, reason);
  Send(body);
}

/* Unlike strtok, this preserves empty fields, which must be rejected. */
static size_t Split(char *body, char *fields[], size_t capacity)
{
  size_t count = 1U;
  fields[0] = body;
  while (*body != '\0')
  {
    if (*body == ',')
    {
      if (count == capacity) return 0U;
      *body = '\0';
      fields[count++] = body + 1;
    }
    ++body;
  }
  return count;
}

static void CalibrationStatus(uint32_t sequence)
{
  char reply[100];
  if (sequence != 0U)
    (void)snprintf(reply, sizeof(reply), "%lu,OK,CAL,%u,%lu,%lu,%u",
                   (unsigned long)sequence, (unsigned)Calibration_Stage(),
                   (unsigned long)Calibration_Samples(),
                   (unsigned long)Calibration_Coverage(), Calibration_Ready() ? 1U : 0U);
  else
    (void)snprintf(reply, sizeof(reply), "0,CALSTAT,%u,%lu,%lu,%u",
                   (unsigned)Calibration_Stage(), (unsigned long)Calibration_Samples(),
                   (unsigned long)Calibration_Coverage(), Calibration_Ready() ? 1U : 0U);
  Send(reply);
}

static bool Signed(const char *text, uint32_t limit, int32_t *result)
{
  bool negative = *text == '-';
  uint32_t value;
  if (!Wire_UInt(text + (negative ? 1 : 0), limit, &value)) return false;
  *result = negative ? -(int32_t)value : (int32_t)value;
  return true;
}

static void HandleCalibration(uint32_t sequence, char *fields[], size_t count)
{
  const char *operation = count >= 3U ? fields[2] : "";
  const char *error = NULL;
  char reply[WIRE_MAX_BODY + 1U];
  if (strcmp(operation, "LOAD") == 0 && count == 10U)
  {
    CalibrationProfile p;
    int32_t values[7];
    unsigned i;
    for (i = 0U; i < 7U; ++i)
      if (!Signed(fields[i + 3U], 32768000U, &values[i]))
      { ReplyError(sequence, "CAL_PROFILE"); return; }
    p.bias_x = values[0] / 1000.0f; p.bias_y = values[1] / 1000.0f;
    p.m00 = values[2] / 1000000.0f; p.m01 = values[3] / 1000000.0f;
    p.m11 = values[4] / 1000000.0f; p.north = values[5]; p.direction = values[6];
    if (!Calibration_Load(&p)) error = "CAL_PROFILE";
  }
  else if (count != 3U) error = "BAD_COMMAND";
  else if (strcmp(operation, "GET") == 0) { /* status only */ }
  else if (strcmp(operation, "READ") == 0)
  {
    const CalibrationProfile *p = Calibration_Profile();
    if (!Calibration_Ready() || Calibration_Busy())
    { ReplyError(sequence, "CAL_STATE"); return; }
    (void)snprintf(reply, sizeof(reply), "%lu,OK,CALDATA,%ld,%ld,%ld,%ld,%ld,%ld,%ld",
                   (unsigned long)sequence, (long)lroundf(p->bias_x * 1000.0f),
                   (long)lroundf(p->bias_y * 1000.0f), (long)lroundf(p->m00 * 1000000.0f),
                   (long)lroundf(p->m01 * 1000000.0f), (long)lroundf(p->m11 * 1000000.0f),
                   (long)p->north, (long)p->direction);
    Send(reply);
    return;
  }
  else if (strcmp(operation, "CANCEL") == 0) Calibration_Cancel();
  else if (strcmp(operation, "RESET") == 0) Calibration_Init();
  else if (!sample_valid || saturated ||
           (uint32_t)(HAL_GetTick() - last_sample) > COMPASS_STALE_MS)
    error = "CAL_SENSOR";
  else if (strcmp(operation, "START") == 0) Calibration_Start();
  else if (strcmp(operation, "FINISH") == 0) error = Calibration_Finish(sensor.sensitivity_nt);
  else if (strcmp(operation, "NORTH") == 0) error = Calibration_SetNorth();
  else if (strcmp(operation, "TURN") == 0) error = Calibration_SetTurn();
  else error = "BAD_COMMAND";
  if (error != NULL) ReplyError(sequence, error);
  else CalibrationStatus(sequence);
  calibration_activity = HAL_GetTick();
  last_frame = HAL_GetTick() - COMPASS_FRAME_MS;
}

static void HandleFrame(void)
{
  char *body, *fields[12], reply[100];
  uint32_t sequence = 0U, requested[3];
  size_t count;
  unsigned i;
  if (!Wire_Decode(rx_frame, &body))
  {
    ReplyError(0U, "BAD_FRAME");
    return;
  }
  count = Split(body, fields, 12U);
  if (count < 2U || !Wire_UInt(fields[0], 65535U, &sequence) || sequence == 0U)
  {
    ReplyError(0U, "BAD_REQUEST");
    return;
  }
  if (strcmp(fields[1], "PING") == 0 && count == 2U)
  {
    (void)snprintf(reply, sizeof(reply), "%lu,PONG,2,%s,%u,%08lX%08lX%08lX",
                   (unsigned long)sequence, Mag_Name(&sensor), sensor.address,
                   (unsigned long)HAL_GetUIDw0(), (unsigned long)HAL_GetUIDw1(),
                   (unsigned long)HAL_GetUIDw2());
    Send(reply);
    return;
  }
  if (strcmp(fields[1], "CAL") == 0)
  {
    HandleCalibration(sequence, fields, count);
    return;
  }
  if (strcmp(fields[1], "SET") == 0 && count == 5U)
  {
    /* Validate all three coefficients before changing any of them. SET is
     * idempotent, so retransmitting after a lost reply is safe. */
    for (i = 0U; i < 3U; ++i)
    {
      if (!Wire_UInt(fields[i + 2U], 200U, &requested[i]))
      {
        ReplyError(sequence, "RANGE");
        return;
      }
    }
    for (i = 0U; i < 3U; ++i) gain[i] = (uint16_t)requested[i];
  }
  else if (!(strcmp(fields[1], "GET") == 0 && count == 2U))
  {
    ReplyError(sequence, "BAD_COMMAND");
    return;
  }
  (void)snprintf(reply, sizeof(reply), "%lu,OK,%s,%u,%u,%u",
                 (unsigned long)sequence, fields[1], gain[0], gain[1], gain[2]);
  Send(reply);
  /* Force the next display snapshot to use the new coefficients. */
  last_frame = HAL_GetTick() - COMPASS_FRAME_MS;
}

static void ProcessUart(void)
{
  unsigned budget = 128U;
  if (rx_restart || rx_overflow)
  {
    uint32_t mask = __get_PRIMASK();
    bool restart;
    __disable_irq();
    restart = rx_restart;
    rx_restart = false;
    rx_overflow = false;
    rx_tail = rx_head;
    __set_PRIMASK(mask);
    collecting = false;
    rx_length = 0U;
    if (restart)
    {
      (void)HAL_UART_AbortReceive(&huart6);
      if (HAL_UART_Receive_IT(&huart6, &irq_byte, 1U) != HAL_OK) rx_restart = true;
    }
    ReplyError(0U, restart ? "UART_ERROR" : "RX_OVERFLOW");
  }
  while (budget-- != 0U && rx_tail != rx_head)
  {
    uint8_t value = rx_buffer[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % RX_CAPACITY);
    if (value == '$')
    {
      collecting = true;
      rx_length = 0U;
    }
    if (!collecting) continue;
    if (value == '\n')
    {
      rx_frame[rx_length] = '\0';
      collecting = false;
      HandleFrame();
    }
    else if (value == '\0' || value > 0x7FU || rx_length >= sizeof(rx_frame) - 1U)
    {
      collecting = false;
      rx_length = 0U;
      ReplyError(0U, "FRAME_TOO_LONG");
    }
    else
      rx_frame[rx_length++] = (char)value;
  }
}

static void Publish(uint32_t now)
{
  CompassSample sample = {0};
  char body[WIRE_MAX_BODY + 1U];
  sample.tick_ms = now;
  memcpy(sample.gain, gain, sizeof(gain));
  sample.heading = -1;
  if (sample_valid && (uint32_t)(now - last_sample) <= COMPASS_STALE_MS)
  {
    Calibration_Field(raw, sensor.sensitivity_nt, gain, sample.field);
    sample.flags |= COMPASS_FLAG_SAMPLE;
    if (saturated)
      sample.flags |= COMPASS_FLAG_SATURATED;
    else
    {
      sample.heading = Calibration_Heading(sample.field);
      if (sample.heading >= 0) sample.flags |= COMPASS_FLAG_HEADING;
    }
  }
  if (Calibration_Ready()) sample.flags |= COMPASS_FLAG_CALIBRATED;
  if (Calibration_Busy()) sample.flags |= COMPASS_FLAG_CALIBRATING;
  if (Oled_Online()) sample.flags |= COMPASS_FLAG_OLED;
  Oled_Render(&sample);
  (void)snprintf(body, sizeof(body), "0,DATA,%lu,%ld,%ld,%ld,%ld,%u,%u,%u,%u",
                 (unsigned long)now, (long)sample.field[0], (long)sample.field[1],
                 (long)sample.field[2], (long)sample.heading,
                 gain[0], gain[1], gain[2], sample.flags);
  Send(body);
  CalibrationStatus(0U);
}

void Compass_Init(void)
{
  uint32_t now;
  rx_head = rx_tail = 0U;
  rx_overflow = rx_restart = false;
  collecting = sample_valid = saturated = false;
  rx_length = 0U;
  last_sample = 0U;
  gain[0] = gain[1] = gain[2] = 100U;
  Calibration_Init();
  /* All I2C transactions, including initialization, are synchronous here. */
  (void)Oled_Init();
  sensor_online = Mag_Init(&sensor);
  now = HAL_GetTick();
  last_sensor_retry = last_oled_retry = now;
  last_poll = now - COMPASS_SAMPLE_MS;
  last_frame = now - COMPASS_FRAME_MS;
  if (HAL_UART_Receive_IT(&huart6, &irq_byte, 1U) != HAL_OK) Error_Handler();
}

void Compass_Process(void)
{
  uint32_t now;
  ProcessUart();
  now = HAL_GetTick();
  if (sensor_online && (uint32_t)(now - last_poll) >= COMPASS_SAMPLE_MS)
  {
    MagResult result;
    last_poll = now;
    result = Mag_Read(&sensor, raw);
    if (result == MAG_READY)
    {
      unsigned i;
      last_sample = HAL_GetTick();
      sample_valid = true;
      saturated = false;
      for (i = 0U; i < 3U; ++i)
        if (raw[i] >= 32760 || raw[i] <= -32760) saturated = true;
      if (!saturated) Calibration_Add(raw);
    }
    else if (result == MAG_ERROR)
    {
      sensor_online = false;
      sample_valid = false;
    }
  }
  now = HAL_GetTick();
  if (Calibration_Busy() &&
      ((uint32_t)(now - calibration_activity) > COMPASS_CAL_TIMEOUT_MS ||
       (uint32_t)(now - last_sample) > COMPASS_STALE_MS || saturated))
  {
    Calibration_Cancel();
    ReplyError(0U, "CAL_ABORTED");
  }
  if ((!sensor_online || (uint32_t)(now - last_sample) > COMPASS_STALE_MS) &&
      (uint32_t)(now - last_sensor_retry) >= COMPASS_RETRY_MS)
  {
    last_sensor_retry = now;
    sample_valid = false;
    sensor_online = Mag_Init(&sensor);
  }
  if (!Oled_Online() && (uint32_t)(now - last_oled_retry) >= COMPASS_RETRY_MS)
  {
    last_oled_retry = now;
    (void)Oled_Init();
  }
  now = HAL_GetTick();
  if ((uint32_t)(now - last_frame) >= COMPASS_FRAME_MS && !Oled_Busy())
  {
    last_frame = now;
    Publish(now);
  }
  Oled_Process();
  HAL_Delay(1U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    uint16_t next = (uint16_t)((rx_head + 1U) % RX_CAPACITY);
    if (next == rx_tail)
      rx_overflow = true;
    else
    {
      rx_buffer[rx_head] = irq_byte;
      rx_head = next;
    }
    if (HAL_UART_Receive_IT(huart, &irq_byte, 1U) != HAL_OK) rx_restart = true;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  /* Recovery and all parsing/transmission stay outside the interrupt. */
  if (huart->Instance == USART6) rx_restart = true;
}
