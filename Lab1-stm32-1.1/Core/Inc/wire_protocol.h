#ifndef WIRE_PROTOCOL_H
#define WIRE_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define WIRE_MAX_BODY 160U
#define WIRE_FRAME_SIZE (WIRE_MAX_BODY + 8U)

uint16_t Wire_Crc16(const char *data, size_t length);
size_t Wire_Encode(const char *body, char *frame, size_t capacity);
/* Decodes in place; on success body is NUL-terminated, without $ and *CRC. */
bool Wire_Decode(char *frame, char **body);
bool Wire_UInt(const char *text, uint32_t limit, uint32_t *value);

#endif
