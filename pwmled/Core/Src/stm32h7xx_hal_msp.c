#include "main.h"

/**
  * @brief  Initializes the TIM PWM MSP.
  * @param  htim TIM PWM handle
  * @retval None
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef tim3OC_ch_gpios;

  // 1. Enable the peripheral clock for the TIM3 peripheral
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();  // Enable GPIOB clock for PB0

  // 2. Configure PB0 to behave as TIM3 CH3 (AF2)
  tim3OC_ch_gpios.Pin = GPIO_PIN_0;  // Use PB0 for PWM output
  tim3OC_ch_gpios.Mode = GPIO_MODE_AF_PP;  // Alternate function push-pull mode
  tim3OC_ch_gpios.Pull = GPIO_PULLDOWN;   // No pull-up or pull-down resistors
  tim3OC_ch_gpios.Speed = GPIO_SPEED_FREQ_LOW;  // Low-speed setting
  tim3OC_ch_gpios.Alternate = GPIO_AF2_TIM3;  // TIM3 alternate function (AF2)
  HAL_GPIO_Init(GPIOB, &tim3OC_ch_gpios);  // Initialize PB0 for PWM output
}
