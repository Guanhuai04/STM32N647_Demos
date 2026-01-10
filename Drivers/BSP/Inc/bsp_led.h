/**
 ****************************************************************************************************
 * @file        led.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-13
 * @brief       LED驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 N647开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#ifndef __LED_H
#define __LED_H

#include <stdint.h>
typedef enum {
  LED1 = 0U,
  LED_RED = LED1,
  LED2 = 1U,
  LED_GREEN = LED2,
  LEDn,
} Led_TypeDef;

/* 引脚定义 */
#define LED1_GPIO_PORT GPIOG
#define LED1_GPIO_PIN GPIO_PIN_10
#define LED2_GPIO_PORT GPIOE
#define LED2_GPIO_PIN GPIO_PIN_10

#define LED1_GPIO_CLK_ENABLE() __HAL_RCC_GPIOG_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE() __HAL_RCC_GPIOG_CLK_DISABLE()

#define LED2_GPIO_CLK_ENABLE() __HAL_RCC_GPIOE_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE() __HAL_RCC_GPIOE_CLK_DISABLE()

void BSP_LED_Init(Led_TypeDef Led);
void BSP_LED_DeInit(Led_TypeDef Led);
void BSP_LED_On(Led_TypeDef Led);
void BSP_LED_Off(Led_TypeDef Led);
void BSP_LED_Toggle(Led_TypeDef Led);
uint32_t BSP_LED_GetState(Led_TypeDef Led);

#endif
