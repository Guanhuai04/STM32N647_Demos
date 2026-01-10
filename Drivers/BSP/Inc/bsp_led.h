#ifndef __BSP_LED_H
#define __BSP_LED_H

#include "stm32n6xx_hal.h"

typedef enum {
  LED1 = 0U,
  LED_RED = LED1,
  LED2 = 1U,
  LED_GREEN = LED2,
  LEDn,
} Led_TypeDef;

typedef enum {
  LED_OFF = 0U,
  LED_ON = 1U,
} LedState_TypeDef;

/* 引脚定义 */
#define BSP_LED1_GPIO_PORT GPIOG
#define BSP_LED1_GPIO_PIN  GPIO_PIN_10
#define BSP_LED2_GPIO_PORT GPIOE
#define BSP_LED2_GPIO_PIN  GPIO_PIN_10

#define BSP_LED1_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOG_CLK_ENABLE()
#define BSP_LED1_GPIO_CLK_DISABLE() __HAL_RCC_GPIOG_CLK_DISABLE()

#define BSP_LED2_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOE_CLK_ENABLE()
#define BSP_LED2_GPIO_CLK_DISABLE() __HAL_RCC_GPIOE_CLK_DISABLE()

void BSP_LED_Init(Led_TypeDef Led);
void BSP_LED_DeInit(Led_TypeDef Led);
void BSP_LED_On(Led_TypeDef Led);
void BSP_LED_Off(Led_TypeDef Led);
void BSP_LED_Toggle(Led_TypeDef Led);
void BSP_LED_SetState(Led_TypeDef Led, LedState_TypeDef State);
LedState_TypeDef BSP_LED_GetState(Led_TypeDef Led);

#endif
