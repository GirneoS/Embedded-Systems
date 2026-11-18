#include "wire_protocol.h"
#include <stdio.h>
#include <string.h>

uint16_t Wire_Crc16(const char *data, size_t length)
{
  uint16_t crc = 0xFFFFU;
  size_t i;
  unsigned bit;
  for (i = 0U; i < length; ++i)
  {
    crc ^= (uint16_t)(uint8_t)data[i] << 8;
    for (bit = 0U; bit < 8U; ++bit)
    {
      crc = (crc & 0x8000U) != 0U ? (uint16_t)((crc << 1) ^ 0x1021U)
                                 : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

size_t Wire_Encode(const char *body, char *frame, size_t capacity)
{
  size_t length = strlen(body);
  int written;
  if (length == 0U || length > WIRE_MAX_BODY || capacity < length + 8U)
  {
    return 0U;
  }
  written = snprintf(frame, capacity, "$%s*%04X\n", body,
                     (unsigned)Wire_Crc16(body, length));
  return written > 0 ? (size_t)written : 0U;
}

static int HexDigit(char c)
{
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

bool Wire_Decode(char *frame, char **body)
{
  char *star;
  size_t length = strlen(frame);
  uint16_t received = 0U;
  unsigned i;
  if (length != 0U && frame[length - 1U] == '\n') frame[--length] = '\0';
  if (length != 0U && frame[length - 1U] == '\r') frame[--length] = '\0';
  if (length < 7U || frame[0] != '$') return false;
  star = strchr(frame + 1, '*');
  if (star == NULL || strlen(star + 1) != 4U ||
      (size_t)(star - frame - 1) > WIRE_MAX_BODY || star == frame + 1)
    return false;
  for (i = 0U; i < 4U; ++i)
  {
    int digit = HexDigit(star[1U + i]);
    if (digit < 0) return false;
    received = (uint16_t)((received << 4) | (unsigned)digit);
  }
  if (received != Wire_Crc16(frame + 1, (size_t)(star - frame - 1))) return false;
  *star = '\0';
  *body = frame + 1;
  return true;
}

bool Wire_UInt(const char *text, uint32_t limit, uint32_t *value)
{
  uint32_t result = 0U;
  if (*text == '\0') return false;
  while (*text != '\0')
  {
    uint32_t digit;
    if (*text < '0' || *text > '9') return false;
    digit = (uint32_t)(*text++ - '0');
    if (digit > limit || result > (limit - digit) / 10U) return false;
    result = result * 10U + digit;
  }
  *value = result;
  return true;
}
