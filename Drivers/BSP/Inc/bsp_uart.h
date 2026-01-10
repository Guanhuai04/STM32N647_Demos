#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "stm32n6xx_hal.h"

HAL_StatusTypeDef BSP_UART_Init_IT(UART_HandleTypeDef *huart, uint8_t *buf,
                                   uint16_t len);
HAL_StatusTypeDef BSP_UART_Init_DMA(UART_HandleTypeDef *huart, uint8_t *buf,
                                    uint16_t max_len);

HAL_StatusTypeDef BSP_UART_Transmit(UART_HandleTypeDef *huart,
                                    const uint8_t *buf, uint16_t len);

__weak void BSP_UART_RxCallback(UART_HandleTypeDef *huart, uint8_t *buf,
                                uint16_t len);
__weak void BSP_UART_TxCallback(UART_HandleTypeDef *huart);
#endif