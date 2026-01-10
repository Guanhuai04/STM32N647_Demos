#include "bsp_led.h"
#include "stm32n6xx_hal_gpio.h"

static GPIO_TypeDef *LED_PORT[LEDn] = {BSP_LED1_GPIO_PORT, BSP_LED2_GPIO_PORT};

static const uint32_t LED_PIN[LEDn] = {BSP_LED1_GPIO_PIN, BSP_LED2_GPIO_PIN};

void BSP_LED_Init(Led_TypeDef Led) {
  /* Enable the GPIO_LED clock */
  uint8_t ret = 1;
  switch (Led) {
  case LED1:
    HAL_PWREx_EnableVddIO2();
    BSP_LED1_GPIO_CLK_ENABLE();
    break;

  case LED2:
    BSP_LED2_GPIO_CLK_ENABLE();
    break;

  default:
    ret = 0;
    break;
  }

  if (ret) {
    GPIO_InitTypeDef gpio_init_structure;
    gpio_init_structure.Pin = LED_PIN[Led];
    gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Pull = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT[Led], &gpio_init_structure);

    BSP_LED_Off(Led);
  }
}

void BSP_LED_DeInit(Led_TypeDef Led) {
  GPIO_InitTypeDef gpio_init_structure;

  BSP_LED_Off(Led);

  gpio_init_structure.Pin = LED_PIN[Led];
  HAL_GPIO_DeInit(LED_PORT[Led], gpio_init_structure.Pin);
}

void BSP_LED_On(Led_TypeDef Led) {
  HAL_GPIO_WritePin(LED_PORT[Led], (uint16_t)LED_PIN[Led], GPIO_PIN_RESET);
}

void BSP_LED_Off(Led_TypeDef Led) {
  HAL_GPIO_WritePin(LED_PORT[Led], (uint16_t)LED_PIN[Led], GPIO_PIN_SET);
}

void BSP_LED_Toggle(Led_TypeDef Led) {
  HAL_GPIO_TogglePin(LED_PORT[Led], (uint16_t)LED_PIN[Led]);
}

void BSP_LED_SetState(Led_TypeDef Led, LedState_TypeDef State) {
  if (State == LED_ON) {
    BSP_LED_On(Led);
  } else {
    BSP_LED_Off(Led);
  }
}

LedState_TypeDef BSP_LED_GetState(Led_TypeDef Led) {
  return (LedState_TypeDef)HAL_GPIO_ReadPin(LED_PORT[Led],
                                            (uint16_t)LED_PIN[Led]);
}
