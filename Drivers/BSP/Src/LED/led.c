/**
 ****************************************************************************************************
 * @file        led.c
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

#include "bsp_led.h"

static GPIO_TypeDef *LED_PORT[LEDn] = { LED1_GPIO_PORT, LED2_GPIO_PORT };

static const uint32_t LED_PIN[LEDn] = { LED1_GPIO_PIN, LED2_GPIO_PIN };

void BSP_LED_Init(Led_TypeDef Led) {
	/* Enable the GPIO_LED clock */
	switch (Led) {
	case LED1:
		HAL_PWREx_EnableVddIO2();
		LED1_GPIO_CLK_ENABLE();
		break;

	case LED2:

		LED2_GPIO_CLK_ENABLE();
		break;
	}

	BSP_LED_Off(Led);
}

void BSP_LED_On(Led_TypeDef Led) {
	if (Led == LED1) {
		/* LED1 High active */
		HAL_GPIO_WritePin(LED_PORT[Led], (uint16_t) LED_PIN[Led], GPIO_PIN_SET);
	} else {
		/* LED2 Low active */
		HAL_GPIO_WritePin(LED_PORT[Led], (uint16_t) LED_PIN[Led], GPIO_PIN_RESET);
	}
}

void BSP_LED_Off(Led_TypeDef Led) {
	if (Led == LED1) {
		/* LED1 High active */
		HAL_GPIO_WritePin(LED_PORT[Led], (uint16_t) LED_PIN[Led], GPIO_PIN_RESET);
	} else {
		/* LED2 Low active */
		HAL_GPIO_WritePin(LED_PORT[Led], (uint16_t) LED_PIN[Led], GPIO_PIN_SET);
	}
}

void BSP_LED_Toggle(Led_TypeDef Led) {
	HAL_GPIO_TogglePin(LED_PORT[Led], (uint16_t) LED_PIN[Led]);
}

uint32_t BSP_LED_GetState(Led_TypeDef Led) {
	uint32_t ret;

	ret = (uint32_t) HAL_GPIO_ReadPin(LED_PORT[Led], (uint16_t) LED_PIN[Led]);
	if (Led == LED2) {
		/* LED2 Low active */
		ret = (uint32_t) ((ret == 1U) ? 0U : 1U);
	}

	return ret;
}
