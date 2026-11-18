#include "fake_hal.h"
#include "compass.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>

UART_HandleTypeDef huart6 = {USART6};
I2C_HandleTypeDef hi2c1;
static uint8_t registers[128][256], present[128], display[1024];
static uint8_t *receive_byte;
static uint32_t ticks;
static char output[65536];
static size_t output_length;
static unsigned page, reset_count, interrupt_depth;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);

void Fake_Reset(unsigned address, unsigned id)
{
  memset(registers, 0, sizeof(registers));
  memset(present, 0, sizeof(present));
  memset(display, 0, sizeof(display));
  present[address] = 1U;
  registers[address][0x0F] = (uint8_t)id;
  registers[address][id == 0x49U ? 0x07 : 0x27] = 8U;
  present[0x70] = present[0x3C] = 1U;
  registers[0x70][1] = 0xA5U;
  registers[0x70][3] = 0xFFU;
  ticks = 0U;
  receive_byte = NULL;
  output_length = 0U;
  output[0] = '\0';
  page = reset_count = interrupt_depth = 0U;
  Compass_Init();
}

void Fake_Present(unsigned address, unsigned value) { present[address] = (uint8_t)value; }
unsigned Fake_Register(unsigned address, unsigned reg) { return registers[address][reg]; }
unsigned Fake_ResetCount(void) { return reset_count; }
const uint8_t *Fake_Display(void) { return display; }

void Fake_Raw(unsigned address, unsigned start, int x, int y, int z)
{
  int values[3] = {x, y, z};
  unsigned i;
  for (i = 0U; i < 3U; ++i)
  {
    registers[address][start + 2U * i] = (uint8_t)values[i];
    registers[address][start + 2U * i + 1U] = (uint8_t)((uint16_t)values[i] >> 8);
  }
}

void Fake_Advance(uint32_t delta) { ticks += delta; }
void Fake_Run(unsigned count) { while (count-- != 0U) Compass_Process(); }
void Fake_Receive(const uint8_t *data, unsigned length)
{
  unsigned i;
  for (i = 0U; i < length; ++i)
  {
    assert(receive_byte != NULL);
    *receive_byte = data[i];
    ++interrupt_depth;
    HAL_UART_RxCpltCallback(&huart6);
    --interrupt_depth;
  }
}
void Fake_UartError(void) { HAL_UART_ErrorCallback(&huart6); }
const char *Fake_Output(void) { output[output_length] = '\0'; return output; }
void Fake_ClearOutput(void) { output_length = 0U; output[0] = '\0'; }

uint32_t HAL_GetTick(void) { return ticks; }
uint32_t HAL_GetUIDw0(void) { return 0x12345678U; }
uint32_t HAL_GetUIDw1(void) { return 0x90ABCDEFU; }
uint32_t HAL_GetUIDw2(void) { return 0x01020304U; }
void HAL_Delay(uint32_t delta) { ticks += delta; }
void Error_Handler(void) { abort(); }
uint32_t __get_PRIMASK(void) { return 0U; }
void __disable_irq(void) {}
void __set_PRIMASK(uint32_t mask) { (void)mask; }

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t n)
{ (void)uart; assert(n == 1U); receive_byte = data; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{ (void)uart; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, const uint8_t *data,
                                    uint16_t n, uint32_t timeout)
{
  (void)uart; (void)timeout;
  assert(interrupt_depth == 0U);
  assert(output_length + n < sizeof(output));
  memcpy(output + output_length, data, n);
  output_length += n;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *bus, uint16_t addr, uint16_t reg,
                                  uint16_t size, uint8_t *data, uint16_t n, uint32_t timeout)
{
  unsigned i;
  (void)bus; (void)size; (void)timeout;
  assert(interrupt_depth == 0U);
  ticks += 1U;
  addr >>= 1;
  if (!present[addr]) return HAL_ERROR;
  for (i = 0U; i < n; ++i) data[i] = registers[addr][(reg & 0x7FU) + i];
  return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *bus, uint16_t addr, uint16_t reg,
                                   uint16_t size, uint8_t *data, uint16_t n, uint32_t timeout)
{
  (void)bus; (void)size; (void)timeout;
  assert(interrupt_depth == 0U);
  ticks += 1U;
  addr >>= 1;
  if (!present[addr]) return HAL_ERROR;
  if (addr == 0x70U && reg == 1U && (data[0] & 1U) == 0U) ++reset_count;
  memcpy(&registers[addr][reg & 0x7FU], data, n);
  if (registers[addr][0x0F] == 0x3DU && reg == 0x21U && (data[0] & 4U))
    registers[addr][0x21U] = 0U; /* Self-clearing SOFT_RST. */
  return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *bus, uint16_t addr,
                                         uint8_t *data, uint16_t n, uint32_t timeout)
{
  (void)bus; (void)timeout;
  assert(interrupt_depth == 0U);
  ticks += n > 100U ? 12U : 1U;
  if (!present[addr >> 1]) return HAL_ERROR;
  if (n == 7U && data[0] == 0U && data[1] == 0x21U) page = data[5];
  if (n == 129U && data[0] == 0x40U && page < 8U)
    memcpy(display + 128U * page, data + 1, 128U);
  return HAL_OK;
}
