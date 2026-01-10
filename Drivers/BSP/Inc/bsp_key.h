#ifndef __BSP_KEY_H
#define __BSP_KEY_H

#include "stm32n6xx.h"

#define KEY_DEBOUNCE_TIME_MS 20U

#define BSP_KEY_UP_GPIO_PIN     GPIO_PIN_13
#define BSP_KEY_UP_GPIO_PORT    GPIOC
#define BSP_KEY_DOWN_GPIO_PIN   GPIO_PIN_1
#define BSP_KEY_DOWN_GPIO_PORT  GPIOD
#define BSP_KEY_LEFT_GPIO_PIN   GPIO_PIN_11
#define BSP_KEY_LEFT_GPIO_PORT  GPIOG
#define BSP_KEY_RIGHT_GPIO_PIN  GPIO_PIN_6
#define BSP_KEY_RIGHT_GPIO_PORT GPIOC

typedef enum {
  KEY_UP = 0U,
  KEY_DOWN = 1U,
  KEY_LEFT = 2U,
  KEY_RIGHT = 3U,
  KEYn,
} Key_TypeDef;

typedef enum { KEY_MODE_GPIO = 0U, KEY_MODE_EXTI = 1U } KeyMode_TypeDef;

uint32_t BSP_KEY_GetState(Key_TypeDef Button);
void BSP_KEY_Callback(Key_TypeDef Button);

#endif