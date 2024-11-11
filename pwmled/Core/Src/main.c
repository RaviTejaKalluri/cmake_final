#include "stm32h7xx_hal.h"
#include "main.h"
#include <string.h>
TIM_HandleTypeDef htimer3;  // Define handle for timer
UART_HandleTypeDef huart2;  // UART handle

/* Private function prototypes */
void GPIO_Init(void);
void Error_handler(void);
void TIMER2_Init(void);
void UART2_Init(void);
void SystemClock_Config_HSE(uint8_t clock_freq);

int main(void)
{
  uint16_t brightness = 0;

  // Initialize HAL and configure system clock
  HAL_Init();
  SystemClock_Config_HSE(SYS_CLOCK_FREQ_50_MHZ);
  GPIO_Init();  // Initialize GPIO for LED (PB0)
  UART2_Init(); // Initialize UART (if necessary)
  TIMER2_Init();  // Initialize Timer for PWM

  // Start PWM on TIM3 channel 3 (PB0)
  if (HAL_TIM_PWM_Start(&htimer3, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_handler();  // Error handling if PWM start fails
  }

  // Main loop: Fade in and fade out the LED
  while (1)
  {
    // Gradually increase the brightness
    while (brightness < htimer3.Init.Period)
    {
      brightness += 20;
      __HAL_TIM_SET_COMPARE(&htimer3, TIM_CHANNEL_3, brightness);  // Update PWM duty cycle
      HAL_Delay(10);  // Small delay for visible effect
    }

    // Gradually decrease the brightness
    while (brightness > 0)
    {
      brightness -= 20;
      __HAL_TIM_SET_COMPARE(&htimer3, TIM_CHANNEL_3, brightness);  // Update PWM duty cycle
      HAL_Delay(10);  // Small delay for visible effect
    }
  }
  return 0;
}

/**
  * @brief System Clock Configuration (for 50 MHz)
  * @retval None
  */
void SystemClock_Config_HSE(uint8_t clock_freq)
{
  RCC_OscInitTypeDef Osc_Init;
  RCC_ClkInitTypeDef Clock_Init;
  uint8_t flash_latency = 0;

  // HSE PLL configuration based on the requested clock frequency
  Osc_Init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  Osc_Init.HSEState = RCC_HSE_ON;
  Osc_Init.PLL.PLLState = RCC_PLL_ON;
  Osc_Init.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  switch (clock_freq)
  {
    case SYS_CLOCK_FREQ_50_MHZ:
      Osc_Init.PLL.PLLM = 4;
      Osc_Init.PLL.PLLN = 50;
      Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
      Osc_Init.PLL.PLLQ = 2;
      Osc_Init.PLL.PLLR = 2;
      Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                             RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
      Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
      Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
      Clock_Init.APB1CLKDivider = RCC_HCLK_DIV2;
      Clock_Init.APB2CLKDivider = RCC_HCLK_DIV1;
      flash_latency = 1;
      break;

    // Add more cases for different frequencies if necessary
    default:
      return;
  }

  // Apply the clock configuration
  if (HAL_RCC_OscConfig(&Osc_Init) != HAL_OK)
  {
    Error_handler();
  }

  if (HAL_RCC_ClockConfig(&Clock_Init, flash_latency) != HAL_OK)
  {
    Error_handler();
  }

  uint32_t hclk_freq = HAL_RCC_GetHCLKFreq();
  HAL_SYSTICK_Config(hclk_freq / 1000);  // Configure Systick for 1ms interval
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief GPIO Initialization (PB0 for PWM)
  * @param None
  * @retval None
  */
void GPIO_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();  // Enable GPIOB clock
  GPIO_InitTypeDef ledgpio;
  ledgpio.Pin = GPIO_PIN_0;  // Use PB0 for PWM output
  ledgpio.Mode = GPIO_MODE_AF_PP;  // Alternate function push-pull mode
  ledgpio.Pull = GPIO_NOPULL;  // No pull-up or pull-down resistors
  ledgpio.Speed = GPIO_SPEED_FREQ_LOW;  // Low-speed setting
  ledgpio.Alternate = GPIO_AF2_TIM3;  // TIM3 alternate function (AF2)
  HAL_GPIO_Init(GPIOB, &ledgpio);  // Initialize PB0 for PWM output
}

/**
  * @brief UART2 Initialization Function (Optional, for debugging)
  * @param None
  * @retval None
  */
void UART2_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_handler();  // Handle UART initialization error
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
void TIMER2_Init(void)
{
  TIM_OC_InitTypeDef tim3PWM_Config;
  htimer3.Instance = TIM3;
  htimer3.Init.Period = 10000 - 1;  // Period for PWM (adjust as necessary)
  htimer3.Init.Prescaler = 4;  // Prescaler value (adjust as necessary)
  if (HAL_TIM_PWM_Init(&htimer3) != HAL_OK)
  {
    Error_handler();  // Handle PWM initialization error
  }

  memset(&tim3PWM_Config, 0, sizeof(tim3PWM_Config));

  tim3PWM_Config.OCMode = TIM_OCMODE_PWM1;
  tim3PWM_Config.OCPolarity = TIM_OCPOLARITY_HIGH;  // Active high polarity
  tim3PWM_Config.Pulse = 0;  // Set initial duty cycle to 0%
  if (HAL_TIM_PWM_ConfigChannel(&htimer3, &tim3PWM_Config, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_handler();  // Handle PWM configuration error
  }
}

/**
  * @brief  Error handler (stuck in infinite loop)
  * @retval None
  */
void Error_handler(void)
{
  while (1);  // Stay here in case of error
}
