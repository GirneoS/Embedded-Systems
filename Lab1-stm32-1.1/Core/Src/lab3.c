#include "lab3.h"

#include "main.h"
#include "tim.h"
#include "usart.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define MODE_COUNT 5U
#define USER_MODE_INDEX 4U
#define USER_STEP_LIMIT 16U
#define UART_RX_BUFFER_SIZE 256U
#define MENU_LINE_SIZE 16U
#define SPEED_MIN_PERCENT 25U
#define SPEED_MAX_PERCENT 400U

typedef enum
{
  COLOR_NONE,
  COLOR_GREEN,
  COLOR_YELLOW,
  COLOR_RED
} LedColor;

typedef struct
{
  uint8_t green;
  uint8_t yellow;
  uint8_t red;
  uint32_t duration_ms;
  bool smooth;
} AnimationStep;

typedef struct
{
  const char *name;
  const AnimationStep *steps;
  uint8_t step_count;
} AnimationMode;

typedef enum
{
  UI_NORMAL,
  UI_MENU_COLOR,
  UI_MENU_BRIGHTNESS,
  UI_MENU_SMOOTH,
  UI_MENU_DURATION,
  UI_MENU_MORE
} UiState;

static const AnimationStep mode1_steps[] = {
  {100U,   0U,   0U, 800U, true},
  {  0U, 100U,   0U, 800U, true},
  {  0U,   0U, 100U, 800U, true}
};

static const AnimationStep mode2_steps[] = {
  {100U,   0U,   0U, 700U, true},
  {  0U,   0U,   0U, 700U, true},
  {  0U,   0U, 100U, 700U, true},
  {  0U,   0U,   0U, 700U, true}
};

static const AnimationStep mode3_steps[] = {
  {100U,   0U,   0U, 1200U, false},
  {  0U,   0U,   0U,  250U, true},
  {  0U, 100U,   0U,  600U, true},
  {  0U,   0U,   0U,  250U, true},
  {  0U,   0U, 100U, 1200U, true},
  {  0U,   0U,   0U,  250U, true}
};

static const AnimationStep mode4_steps[] = {
  {100U,   0U,   0U, 500U, true},
  { 20U, 100U,   0U, 500U, true},
  {  0U,   0U, 100U, 500U, true},
  { 80U,   0U,   0U, 500U, true},
  {  0U, 100U,   0U, 500U, true},
  {  0U,   0U,   0U, 300U, true}
};

static AnimationStep user_steps[USER_STEP_LIMIT] = {
  {100U,   0U,   0U, 600U, true},
  {  0U,   0U, 100U, 600U, true}
};
static uint8_t user_step_count = 2U;

static const AnimationMode preset_modes[4] = {
  {"Smooth circle", mode1_steps, (uint8_t)(sizeof(mode1_steps) / sizeof(mode1_steps[0]))},
  {"Breathing", mode2_steps, (uint8_t)(sizeof(mode2_steps) / sizeof(mode2_steps[0]))},
  {"Traffic light", mode3_steps, (uint8_t)(sizeof(mode3_steps) / sizeof(mode3_steps[0]))},
  {"Wave", mode4_steps, (uint8_t)(sizeof(mode4_steps) / sizeof(mode4_steps[0]))}
};

static volatile uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t uart_rx_head;
static volatile uint16_t uart_rx_tail;
static volatile bool uart_rx_overflow;
static uint8_t uart_irq_byte;

static UiState ui_state = UI_NORMAL;
static char menu_line[MENU_LINE_SIZE];
static uint8_t menu_line_length;
static bool ignore_next_lf;

static AnimationStep draft_steps[USER_STEP_LIMIT];
static uint8_t draft_step_count;
static AnimationStep draft_step;
static LedColor draft_color;

static uint8_t active_mode;
static uint8_t active_step;
static uint16_t speed_percent = 100U;
static uint32_t step_started_ms;
static uint32_t step_duration_ms;
static uint8_t start_green;
static uint8_t start_yellow;
static uint8_t start_red;
static uint8_t current_green;
static uint8_t current_yellow;
static uint8_t current_red;

static void UART_Write(const char *text)
{
  (void)HAL_UART_Transmit(&huart6, (const uint8_t *)text,
                          (uint16_t)strlen(text), HAL_MAX_DELAY);
}

static void UART_WriteChar(uint8_t value)
{
  (void)HAL_UART_Transmit(&huart6, &value, 1U, HAL_MAX_DELAY);
}

static void UART_WriteUInt(uint32_t value)
{
  char digits[10];
  uint8_t length = 0U;

  do
  {
    digits[length++] = (char)('0' + (value % 10U));
    value /= 10U;
  } while ((value != 0U) && (length < sizeof(digits)));

  while (length != 0U)
  {
    UART_WriteChar((uint8_t)digits[--length]);
  }
}

static const char *ColorName(LedColor color)
{
  switch (color)
  {
    case COLOR_GREEN:
      return "green";
    case COLOR_YELLOW:
      return "yellow";
    case COLOR_RED:
      return "red";
    default:
      return "off";
  }
}

static LedColor StepColor(const AnimationStep *step)
{
  if (step->green != 0U)
  {
    return COLOR_GREEN;
  }
  if (step->yellow != 0U)
  {
    return COLOR_YELLOW;
  }
  if (step->red != 0U)
  {
    return COLOR_RED;
  }
  return COLOR_NONE;
}

static uint8_t StepBrightness(const AnimationStep *step)
{
  if (step->green != 0U)
  {
    return step->green;
  }
  if (step->yellow != 0U)
  {
    return step->yellow;
  }
  return step->red;
}

static void UART_PrintStep(const AnimationStep *step, uint8_t number)
{
  UART_Write("  step ");
  UART_WriteUInt(number);
  UART_Write(": color=");
  UART_Write(ColorName(StepColor(step)));
  UART_Write(", brightness=");
  UART_WriteUInt(StepBrightness(step));
  UART_Write("%, transition=");
  UART_Write(step->smooth ? "smooth" : "instant");
  UART_Write(", duration=");
  UART_WriteUInt(step->duration_ms);
  UART_Write(" ms\r\n");
}

static void UART_PrintUserMode(void)
{
  uint8_t i;

  UART_Write("\r\nSaved user mode:\r\n");
  for (i = 0U; i < user_step_count; ++i)
  {
    UART_PrintStep(&user_steps[i], (uint8_t)(i + 1U));
  }
  UART_Write("> ");
}

static const AnimationStep *CurrentSteps(void)
{
  return active_mode == USER_MODE_INDEX
      ? user_steps
      : preset_modes[active_mode].steps;
}

static uint8_t CurrentStepCount(void)
{
  return active_mode == USER_MODE_INDEX
      ? user_step_count
      : preset_modes[active_mode].step_count;
}

static const char *CurrentModeName(void)
{
  return active_mode == USER_MODE_INDEX
      ? "User"
      : preset_modes[active_mode].name;
}

static uint32_t BrightnessToCompare(uint8_t brightness)
{
  uint32_t timer_counts = __HAL_TIM_GET_AUTORELOAD(&htim4) + 1U;
  return (timer_counts * brightness + 50U) / 100U;
}

static void PWM_Set(uint8_t green, uint8_t yellow, uint8_t red)
{
  current_green = green;
  current_yellow = yellow;
  current_red = red;

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, BrightnessToCompare(green));
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, BrightnessToCompare(yellow));
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, BrightnessToCompare(red));
}

static uint32_t ScaledDuration(uint32_t duration_ms)
{
  uint32_t scaled = (duration_ms * 100U + (speed_percent / 2U)) / speed_percent;
  return scaled == 0U ? 1U : scaled;
}

static uint8_t Interpolate(uint8_t from, uint8_t to,
                           uint32_t elapsed, uint32_t duration)
{
  int32_t delta = (int32_t)to - (int32_t)from;
  int32_t value = (int32_t)from
      + (int32_t)((delta * (int32_t)elapsed) / (int32_t)duration);

  if (value < 0)
  {
    return 0U;
  }
  if (value > 100)
  {
    return 100U;
  }
  return (uint8_t)value;
}

static void Animation_StartStep(uint32_t now)
{
  const AnimationStep *step = &CurrentSteps()[active_step];

  start_green = current_green;
  start_yellow = current_yellow;
  start_red = current_red;
  step_started_ms = now;
  step_duration_ms = ScaledDuration(step->duration_ms);

  if (!step->smooth)
  {
    PWM_Set(step->green, step->yellow, step->red);
  }
}

static void Animation_ActivateMode(uint8_t mode)
{
  active_mode = mode;
  active_step = 0U;
  Animation_StartStep(HAL_GetTick());

  UART_Write("\r\nMode ");
  UART_WriteUInt((uint32_t)active_mode + 1U);
  UART_Write(" activated: ");
  UART_Write(CurrentModeName());
  UART_Write(", speed=");
  UART_WriteUInt(speed_percent);
  UART_Write("%\r\n> ");
}

static void Animation_Update(uint32_t now)
{
  const AnimationStep *step = &CurrentSteps()[active_step];
  uint32_t elapsed = (uint32_t)(now - step_started_ms);

  if (elapsed >= step_duration_ms)
  {
    PWM_Set(step->green, step->yellow, step->red);
    active_step = (uint8_t)((active_step + 1U) % CurrentStepCount());
    Animation_StartStep(now);
    return;
  }

  if (step->smooth)
  {
    PWM_Set(Interpolate(start_green, step->green, elapsed, step_duration_ms),
            Interpolate(start_yellow, step->yellow, elapsed, step_duration_ms),
            Interpolate(start_red, step->red, elapsed, step_duration_ms));
  }
}

static void Animation_ChangeSpeed(bool faster)
{
  uint32_t new_speed;

  if (faster)
  {
    new_speed = ((uint32_t)speed_percent * 110U + 50U) / 100U;
    if (new_speed > SPEED_MAX_PERCENT)
    {
      new_speed = SPEED_MAX_PERCENT;
    }
  }
  else
  {
    new_speed = ((uint32_t)speed_percent * 90U + 50U) / 100U;
    if (new_speed < SPEED_MIN_PERCENT)
    {
      new_speed = SPEED_MIN_PERCENT;
    }
  }

  speed_percent = (uint16_t)new_speed;
  Animation_StartStep(HAL_GetTick());

  UART_Write("\r\nAnimation speed: ");
  UART_WriteUInt(speed_percent);
  UART_Write("%\r\n> ");
}

static void UART_RxPush(uint8_t value)
{
  uint16_t next = (uint16_t)((uart_rx_head + 1U) % UART_RX_BUFFER_SIZE);

  if (next == uart_rx_tail)
  {
    uart_rx_overflow = true;
    return;
  }

  uart_rx_buffer[uart_rx_head] = value;
  uart_rx_head = next;
}

static bool UART_RxPop(uint8_t *value)
{
  if (uart_rx_tail == uart_rx_head)
  {
    return false;
  }

  *value = uart_rx_buffer[uart_rx_tail];
  uart_rx_tail = (uint16_t)((uart_rx_tail + 1U) % UART_RX_BUFFER_SIZE);
  return true;
}

static bool ParseUInt(const char *text, uint32_t *value)
{
  uint32_t result = 0U;
  size_t i;

  if (text[0] == '\0')
  {
    return false;
  }

  for (i = 0U; text[i] != '\0'; ++i)
  {
    if ((text[i] < '0') || (text[i] > '9'))
    {
      return false;
    }
    result = result * 10U + (uint32_t)(text[i] - '0');
    if (result > 100000U)
    {
      return false;
    }
  }

  *value = result;
  return true;
}

static void Menu_PrintPrompt(void)
{
  switch (ui_state)
  {
    case UI_MENU_COLOR:
      UART_Write("Color [g=green, y=yellow, r=red, n=off]: ");
      break;
    case UI_MENU_BRIGHTNESS:
      UART_Write("Brightness [0..100]: ");
      break;
    case UI_MENU_SMOOTH:
      UART_Write("Smooth transition [y/n]: ");
      break;
    case UI_MENU_DURATION:
      UART_Write("Duration [10..60000 ms]: ");
      break;
    case UI_MENU_MORE:
      UART_Write("Add another step [a], save [s], cancel [c]: ");
      break;
    default:
      break;
  }
}

static void Menu_Start(void)
{
  draft_step_count = 0U;
  memset(draft_steps, 0, sizeof(draft_steps));
  memset(&draft_step, 0, sizeof(draft_step));
  ui_state = UI_MENU_COLOR;

  UART_Write("\r\nUser mode setup. Up to 16 steps; use at least two colors.\r\n");
  Menu_PrintPrompt();
}

static uint8_t Menu_ColorMask(void)
{
  uint8_t mask = 0U;
  uint8_t i;

  for (i = 0U; i < draft_step_count; ++i)
  {
    if (draft_steps[i].green != 0U)
    {
      mask |= 1U;
    }
    if (draft_steps[i].yellow != 0U)
    {
      mask |= 2U;
    }
    if (draft_steps[i].red != 0U)
    {
      mask |= 4U;
    }
  }
  return mask;
}

static bool HasAtLeastTwoColors(uint8_t mask)
{
  return (mask != 0U) && ((mask & (uint8_t)(mask - 1U)) != 0U);
}

static void Menu_Save(void)
{
  uint8_t i;

  if ((draft_step_count < 2U) || !HasAtLeastTwoColors(Menu_ColorMask()))
  {
    UART_Write("Cannot save: add at least two steps using two different colors.\r\n");
    Menu_PrintPrompt();
    return;
  }

  for (i = 0U; i < draft_step_count; ++i)
  {
    user_steps[i] = draft_steps[i];
  }
  user_step_count = draft_step_count;
  ui_state = UI_NORMAL;

  UART_Write("User mode saved.\r\n");
  Animation_ActivateMode(USER_MODE_INDEX);
}

static void Menu_ProcessLine(const char *line)
{
  uint32_t value;

  switch (ui_state)
  {
    case UI_MENU_COLOR:
      if ((line[0] == '\0') || (line[1] != '\0')
          || ((line[0] != 'g') && (line[0] != 'y')
              && (line[0] != 'r') && (line[0] != 'n')))
      {
        UART_Write("Invalid color.\r\n");
        Menu_PrintPrompt();
        return;
      }

      memset(&draft_step, 0, sizeof(draft_step));
      if (line[0] == 'g')
      {
        draft_color = COLOR_GREEN;
      }
      else if (line[0] == 'y')
      {
        draft_color = COLOR_YELLOW;
      }
      else if (line[0] == 'r')
      {
        draft_color = COLOR_RED;
      }
      else
      {
        draft_color = COLOR_NONE;
      }

      UART_Write("Selected color: ");
      UART_Write(ColorName(draft_color));
      UART_Write("\r\n");
      ui_state = UI_MENU_BRIGHTNESS;
      Menu_PrintPrompt();
      break;

    case UI_MENU_BRIGHTNESS:
      if (!ParseUInt(line, &value) || (value > 100U))
      {
        UART_Write("Invalid brightness.\r\n");
        Menu_PrintPrompt();
        return;
      }

      if (draft_color == COLOR_GREEN)
      {
        draft_step.green = (uint8_t)value;
      }
      else if (draft_color == COLOR_YELLOW)
      {
        draft_step.yellow = (uint8_t)value;
      }
      else if (draft_color == COLOR_RED)
      {
        draft_step.red = (uint8_t)value;
      }

      UART_Write("Brightness: ");
      UART_WriteUInt(value);
      UART_Write("%\r\n");
      ui_state = UI_MENU_SMOOTH;
      Menu_PrintPrompt();
      break;

    case UI_MENU_SMOOTH:
      if ((line[0] == '\0') || (line[1] != '\0')
          || ((line[0] != 'y') && (line[0] != 'n')))
      {
        UART_Write("Enter y or n.\r\n");
        Menu_PrintPrompt();
        return;
      }

      draft_step.smooth = line[0] == 'y';
      UART_Write(draft_step.smooth ? "Transition: smooth\r\n"
                                   : "Transition: instant\r\n");
      ui_state = UI_MENU_DURATION;
      Menu_PrintPrompt();
      break;

    case UI_MENU_DURATION:
      if (!ParseUInt(line, &value) || (value < 10U) || (value > 60000U))
      {
        UART_Write("Invalid duration.\r\n");
        Menu_PrintPrompt();
        return;
      }

      draft_step.duration_ms = value;
      draft_steps[draft_step_count] = draft_step;
      ++draft_step_count;
      UART_PrintStep(&draft_step, draft_step_count);
      ui_state = UI_MENU_MORE;

      if (draft_step_count == USER_STEP_LIMIT)
      {
        UART_Write("Step limit reached. Save [s] or cancel [c]: ");
      }
      else
      {
        Menu_PrintPrompt();
      }
      break;

    case UI_MENU_MORE:
      if ((line[0] == 'a') && (line[1] == '\0')
          && (draft_step_count < USER_STEP_LIMIT))
      {
        ui_state = UI_MENU_COLOR;
        Menu_PrintPrompt();
      }
      else if ((line[0] == 's') && (line[1] == '\0'))
      {
        Menu_Save();
      }
      else if ((line[0] == 'c') && (line[1] == '\0'))
      {
        ui_state = UI_NORMAL;
        UART_Write("Setup cancelled.\r\n> ");
      }
      else
      {
        UART_Write("Invalid action.\r\n");
        Menu_PrintPrompt();
      }
      break;

    default:
      break;
  }
}

static void Normal_ProcessChar(uint8_t value)
{
  switch (value)
  {
    case '1':
      UART_Write("1");
      Animation_ActivateMode((uint8_t)((active_mode + 1U) % MODE_COUNT));
      break;
    case '2':
      UART_Write("2");
      Animation_ActivateMode((uint8_t)((active_mode + MODE_COUNT - 1U) % MODE_COUNT));
      break;
    case '3':
      UART_Write("3");
      Animation_ChangeSpeed(true);
      break;
    case '4':
      UART_Write("4");
      Animation_ChangeSpeed(false);
      break;
    case '5':
      UART_Write("5");
      UART_PrintUserMode();
      break;
    case '\r':
    case '\n':
      if ((value == '\n') && ignore_next_lf)
      {
        ignore_next_lf = false;
        return;
      }
      ignore_next_lf = value == '\r';
      Menu_Start();
      break;
    default:
      if ((value >= 0x20U) && (value <= 0x7EU))
      {
        UART_WriteChar(value);
        UART_Write("\r\nUnknown key. Use 1, 2, 3, 4, 5 or Enter.\r\n> ");
      }
      break;
  }
}

static void Menu_ProcessChar(uint8_t value)
{
  if ((value == '\r') || (value == '\n'))
  {
    if ((value == '\n') && ignore_next_lf)
    {
      ignore_next_lf = false;
      return;
    }

    ignore_next_lf = value == '\r';
    UART_Write("\r\n");
    menu_line[menu_line_length] = '\0';
    Menu_ProcessLine(menu_line);
    menu_line_length = 0U;
    return;
  }

  ignore_next_lf = false;
  if ((value == '\b') || (value == 0x7FU))
  {
    if (menu_line_length != 0U)
    {
      --menu_line_length;
      UART_Write("\b \b");
    }
    return;
  }

  if ((value >= 0x20U) && (value <= 0x7EU))
  {
    if (menu_line_length < (MENU_LINE_SIZE - 1U))
    {
      menu_line[menu_line_length++] = (char)value;
      UART_WriteChar(value);
    }
    else
    {
      menu_line_length = 0U;
      UART_Write("\r\nInput is too long.\r\n");
      Menu_PrintPrompt();
    }
  }
}

static void UART_Process(void)
{
  uint8_t value;

  if (uart_rx_overflow)
  {
    uart_rx_overflow = false;
    UART_Write("\r\nUART input overflow.\r\n> ");
  }

  while (UART_RxPop(&value))
  {
    if (ui_state == UI_NORMAL)
    {
      Normal_ProcessChar(value);
    }
    else
    {
      Menu_ProcessChar(value);
    }
  }
}

void Lab3_Init(void)
{
  PWM_Set(0U, 0U, 0U);

  if ((HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2) != HAL_OK)
      || (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3) != HAL_OK)
      || (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4) != HAL_OK))
  {
    Error_Handler();
  }

  if (HAL_UART_Receive_IT(&huart6, &uart_irq_byte, 1U) != HAL_OK)
  {
    Error_Handler();
  }

  active_mode = 0U;
  active_step = 0U;
  Animation_StartStep(HAL_GetTick());

  UART_Write("\r\nLab 3 garland ready. PWM=4000 Hz, UART=38400 8N1.\r\n");
  UART_Write("1-next, 2-previous, 3-faster, 4-slower, 5-print user mode, Enter-setup.\r\n");
  UART_Write("Mode 1 activated: Smooth circle, speed=100%\r\n> ");
}

void Lab3_Process(void)
{
  UART_Process();
  Animation_Update(HAL_GetTick());
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    UART_RxPush(uart_irq_byte);
    (void)HAL_UART_Receive_IT(&huart6, &uart_irq_byte, 1U);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    (void)HAL_UART_AbortReceive(huart);
    (void)HAL_UART_Receive_IT(&huart6, &uart_irq_byte, 1U);
  }
}
