#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "stm32n6xx_hal.h"

#define BSP_UART_DEBUG_HUART huart1
extern UART_HandleTypeDef BSP_UART_DEBUG_HUART;

typedef enum {
  UART_MODE_BASIC = 0u,
  UART_MODE_IT = 1u,
  UART_MODE_DMA = 2u,
} BSP_UART_Mode_t;

void BSP_UART_Init(UART_HandleTypeDef *huart, BSP_UART_Mode_t mode);

HAL_StatusTypeDef BSP_UART_Register_Receive_Buffer(UART_HandleTypeDef *huart,
                                                   uint8_t *buf, uint16_t len);

HAL_StatusTypeDef BSP_UART_Transmit(UART_HandleTypeDef *huart,
                                    const uint8_t *buf, uint16_t len);

__weak void BSP_UART_RxCallback(UART_HandleTypeDef *huart, uint8_t *buf,
                                uint16_t len);
__weak void BSP_UART_TxCallback(UART_HandleTypeDef *huart);
#endif