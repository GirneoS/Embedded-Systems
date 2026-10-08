/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint16_t states[8];
  uint32_t period_ms;
  uint8_t length;
  bool valid;
} LedSequence;

typedef enum
{
  INPUT_COMMAND,
  INPUT_PERIOD
} InputMode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_PINS (GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)
#define LED_GREEN GPIO_PIN_13
#define LED_YELLOW GPIO_PIN_14
#define LED_RED GPIO_PIN_15
#define LED_NONE 0U

#define UART_RX_BUFFER_SIZE 128U
#define COMMAND_BUFFER_SIZE 40U
#define BUTTON_DEBOUNCE_MS 50U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static LedSequence sequences[8] = {
  {{LED_GREEN, LED_RED, LED_YELLOW, LED_NONE}, 200U, 4U, true},
  {{LED_GREEN, LED_YELLOW, LED_RED, LED_YELLOW}, 500U, 4U, true},
  {{LED_GREEN, LED_NONE, LED_RED, LED_NONE, LED_YELLOW, LED_NONE}, 1000U, 6U, true},
  {{LED_RED, LED_YELLOW, LED_GREEN, LED_YELLOW, LED_NONE}, 750U, 5U, true}
};

static volatile uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t uart_rx_head;
static volatile uint16_t uart_rx_tail;
static uint8_t uart_irq_byte;
static bool button_event;

static char command_buffer[COMMAND_BUFFER_SIZE];
static uint8_t command_length;
static bool previous_was_cr;
static InputMode input_mode = INPUT_COMMAND;

static uint16_t draft_states[8];
static uint8_t draft_length;
static uint8_t next_user_slot = 4U;

static uint8_t active_sequence;
static uint8_t active_step;
static uint32_t last_step_ms;
static uint32_t last_button_ms;
static bool interrupts_enabled;
static bool previous_button_level;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Interrupts_SetEnabled(bool enabled);
static void UART_RxPush(uint8_t value);
static bool UART_RxPop(uint8_t *value);
static void UART_Poll(void);
static void UART_Write(const char *text);
static void UART_WriteChar(uint8_t value);
static void UART_ProcessInput(void);
static void HandleCommand(char *command);
static bool DecodeState(char value, uint16_t *pin);
static bool ParsePeriod(const char *text, uint32_t *period_ms);
static void SaveDraftSequence(uint32_t period_ms);
static void ShowStep(void);
static void ActivateSequence(uint8_t index);
static void ActivateNextSequence(void);
static void Button_Poll(void);
static void Button_Process(void);
static void Garland_Update(uint32_t now);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Interrupts_SetEnabled(bool enabled)
{
  if (enabled == interrupts_enabled)
  {
    return;
  }

  if (enabled)
  {
    interrupts_enabled = true;
    if (HAL_UART_Receive_IT(&huart6, &uart_irq_byte, 1U) != HAL_OK)
    {
      Error_Handler();
    }
  }
  else
  {
    interrupts_enabled = false;
    (void)HAL_UART_AbortReceive(&huart6);
  }
}

static void UART_RxPush(uint8_t value)
{
  uint16_t next = (uint16_t)((uart_rx_head + 1U) % UART_RX_BUFFER_SIZE);

  if (next != uart_rx_tail)
  {
    uart_rx_buffer[uart_rx_head] = value;
    uart_rx_head = next;
  }
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

static void UART_Poll(void)
{
  uint8_t value;

  if (interrupts_enabled)
  {
    return;
  }

  if (HAL_UART_Receive(&huart6, &value, 1U, 0U) == HAL_OK)
  {
    UART_RxPush(value);
  }
}

static void UART_Write(const char *text)
{
  (void)HAL_UART_Transmit(&huart6, (const uint8_t *)text, (uint16_t)strlen(text), HAL_MAX_DELAY);
}

static void UART_WriteChar(uint8_t value)
{
  (void)HAL_UART_Transmit(&huart6, &value, 1U, HAL_MAX_DELAY);
}

static bool DecodeState(char value, uint16_t *pin)
{
  switch (value)
  {
    case 'g':
      *pin = LED_GREEN;
      return true;
    case 'r':
      *pin = LED_RED;
      return true;
    case 'y':
      *pin = LED_YELLOW;
      return true;
    case 'n':
      *pin = LED_NONE;
      return true;
    default:
      return false;
  }
}

static bool ParsePeriod(const char *text, uint32_t *period_ms)
{
  if ((strcmp(text, "fast") == 0) || (strcmp(text, "200") == 0))
  {
    *period_ms = 200U;
    return true;
  }
  if ((strcmp(text, "medium") == 0) || (strcmp(text, "500") == 0))
  {
    *period_ms = 500U;
    return true;
  }
  if ((strcmp(text, "slow") == 0) || (strcmp(text, "1000") == 0))
  {
    *period_ms = 1000U;
    return true;
  }
  return false;
}

static void SaveDraftSequence(uint32_t period_ms)
{
  uint8_t slot = next_user_slot;
  uint8_t i;

  for (i = 0U; i < draft_length; ++i)
  {
    sequences[slot].states[i] = draft_states[i];
  }
  sequences[slot].length = draft_length;
  sequences[slot].period_ms = period_ms;
  sequences[slot].valid = true;

  next_user_slot = (uint8_t)(4U + ((slot - 4U + 1U) % 4U));

  UART_Write("Saved as sequence ");
  UART_WriteChar((uint8_t)('1' + slot));
  UART_Write(".\r\n> ");
}

static void ShowStep(void)
{
  uint16_t pin = sequences[active_sequence].states[active_step];

  HAL_GPIO_WritePin(GPIOD, LED_PINS, GPIO_PIN_RESET);
  if (pin != LED_NONE)
  {
    HAL_GPIO_WritePin(GPIOD, pin, GPIO_PIN_SET);
  }
}

static void ActivateSequence(uint8_t index)
{
  active_sequence = index;
  active_step = 0U;
  last_step_ms = HAL_GetTick();
  ShowStep();

  UART_Write("Active sequence: ");
  UART_WriteChar((uint8_t)('1' + index));
  UART_Write(".\r\n> ");
}

static void ActivateNextSequence(void)
{
  uint8_t offset;

  for (offset = 1U; offset <= 8U; ++offset)
  {
    uint8_t candidate = (uint8_t)((active_sequence + offset) % 8U);
    if (sequences[candidate].valid)
    {
      ActivateSequence(candidate);
      return;
    }
  }
}

static void HandleCommand(char *command)
{
  uint32_t period_ms;
  size_t length;
  uint8_t i;

  if (input_mode == INPUT_PERIOD)
  {
    if (ParsePeriod(command, &period_ms))
    {
      SaveDraftSequence(period_ms);
      input_mode = INPUT_COMMAND;
    }
    else
    {
      UART_Write("Invalid period. Use fast, medium, slow, 200, 500 or 1000.\r\nperiod> ");
    }
    return;
  }

  if (strncmp(command, "new ", 4U) == 0)
  {
    length = strlen(command + 4U);
    if ((length < 2U) || (length > 8U))
    {
      UART_Write("A sequence must contain 2..8 states.\r\n> ");
      return;
    }

    for (i = 0U; i < (uint8_t)length; ++i)
    {
      if (!DecodeState(command[4U + i], &draft_states[i]))
      {
        UART_Write("Invalid state. Use only g, r, y and n.\r\n> ");
        return;
      }
    }

    draft_length = (uint8_t)length;
    input_mode = INPUT_PERIOD;
    UART_Write("Choose period: fast (200), medium (500), slow (1000).\r\nperiod> ");
    return;
  }

  if (strcmp(command, "set interrupts on") == 0)
  {
    Interrupts_SetEnabled(true);
    UART_Write("Interrupt mode enabled.\r\n> ");
    return;
  }

  if (strcmp(command, "set interrupts off") == 0)
  {
    UART_Write("Interrupt mode disabled; UART and button use polling.\r\n> ");
    Interrupts_SetEnabled(false);
    return;
  }

  if ((strncmp(command, "set ", 4U) == 0) && (strlen(command) == 5U)
      && (command[4] >= '1') && (command[4] <= '8'))
  {
    uint8_t index = (uint8_t)(command[4] - '1');
    if (sequences[index].valid)
    {
      ActivateSequence(index);
    }
    else
    {
      UART_Write("This sequence has not been created yet.\r\n> ");
    }
    return;
  }

  UART_Write("Unknown command. Use: new <states>, set <1..8>, set interrupts on/off.\r\n> ");
}

static void UART_ProcessInput(void)
{
  uint8_t value;

  while (UART_RxPop(&value))
  {
    if ((value == '\r') || (value == '\n'))
    {
      if ((value == '\n') && previous_was_cr)
      {
        previous_was_cr = false;
        continue;
      }

      previous_was_cr = value == '\r';
      UART_Write("\r\n");
      if (command_length != 0U)
      {
        command_buffer[command_length] = '\0';
        HandleCommand(command_buffer);
        command_length = 0U;
      }
      else
      {
        UART_Write(input_mode == INPUT_PERIOD ? "period> " : "> ");
      }
      continue;
    }

    previous_was_cr = false;
    if ((value == '\b') || (value == 0x7FU))
    {
      if (command_length != 0U)
      {
        --command_length;
        UART_Write("\b \b");
      }
      continue;
    }

    if ((value >= 0x20U) && (value <= 0x7EU))
    {
      if (command_length < (COMMAND_BUFFER_SIZE - 1U))
      {
        command_buffer[command_length++] = (char)value;
        UART_WriteChar(value);
      }
      else
      {
        command_length = 0U;
        UART_Write("\r\nCommand is too long.\r\n> ");
      }
    }
  }
}

static void Button_Poll(void)
{
  bool button_level = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_SET;

  if (button_level && !previous_button_level)
  {
    button_event = true;
  }
  previous_button_level = button_level;
}

static void Button_Process(void)
{
  uint32_t now;

  if (!button_event)
  {
    return;
  }

  button_event = false;
  now = HAL_GetTick();
  if ((uint32_t)(now - last_button_ms) >= BUTTON_DEBOUNCE_MS)
  {
    last_button_ms = now;
    ActivateNextSequence();
  }
}

static void Garland_Update(uint32_t now)
{
  LedSequence *sequence = &sequences[active_sequence];

  if ((uint32_t)(now - last_step_ms) >= sequence->period_ms)
  {
    last_step_ms = now;
    active_step = (uint8_t)((active_step + 1U) % sequence->length);
    ShowStep();
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  previous_button_level = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_SET;
  Interrupts_SetEnabled(true);
  last_step_ms = HAL_GetTick();
  ShowStep();
  UART_Write("\r\nGarland ready. UART6 38400 8N1.\r\n");
  UART_Write("Commands: new <2..8 states>, set <1..8>, set interrupts on/off.\r\n> ");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    UART_Poll();
    UART_ProcessInput();
    Button_Poll();
    Button_Process();
    Garland_Update(HAL_GetTick());
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    UART_RxPush(uart_irq_byte);
    if (interrupts_enabled)
    {
      (void)HAL_UART_Receive_IT(&huart6, &uart_irq_byte, 1U);
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart->Instance == USART6) && interrupts_enabled)
  {
    (void)HAL_UART_Receive_IT(&huart6, &uart_irq_byte, 1U);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
