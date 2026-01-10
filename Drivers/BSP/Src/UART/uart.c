#include "bsp_uart.h"
#include <stdint.h>
#include <stdio.h>

int __io_putchar(int ch) {
  HAL_UART_Transmit(&BSP_UART_DEBUG_HUART, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

#define BSP_UART_MAX 5u

typedef struct {
  UART_HandleTypeDef *huart;
  uint8_t *buf;
  uint16_t len;
  uint8_t mode;
} UART_Slot_t;

static uint8_t slots_num = 0;
static UART_Slot_t slots[BSP_UART_MAX];

static void slot_add(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t len,
                     uint8_t mode) {
  if (slots_num < BSP_UART_MAX) {
    slots[slots_num].huart = huart;
    slots[slots_num].buf = buf;
    slots[slots_num].len = len;
    slots[slots_num].mode = mode;
    slots_num++;
  }
}

static UART_Slot_t *slot_get(UART_HandleTypeDef *huart) {
  for (uint8_t i = 0; i < slots_num; i++) {
    if (slots[i].huart == huart) {
      return &slots[i];
    }
  }
  return NULL;
}

static void dcache_invalidate(void *addr, uint32_t size) {
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  if (!addr || !size)
    return;
  uintptr_t start = ((uintptr_t)addr) & ~(uintptr_t)31u;
  uintptr_t end = (((uintptr_t)addr) + size + 31u) & ~(uintptr_t)31u;
  SCB_InvalidateDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
#else
  UNUSED(addr);
  UNUSED(size);
#endif
}

static void dcache_clean(const void *addr, uint32_t size) {
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  if (!addr || !size)
    return;
  uintptr_t start = ((uintptr_t)addr) & ~(uintptr_t)31u;
  uintptr_t end = (((uintptr_t)addr) + size + 31u) & ~(uintptr_t)31u;
  SCB_CleanDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
#else
  UNUSED(addr);
  UNUSED(size);
#endif
}

void BSP_UART_Init(UART_HandleTypeDef *huart, BSP_UART_Mode_t mode) {
  slot_add(huart, NULL, 0, mode);
}

HAL_StatusTypeDef BSP_UART_Register_Receive_Buffer(UART_HandleTypeDef *huart,
                                                   uint8_t *buf, uint16_t len) {
  UART_Slot_t *slot = slot_get(huart);
  if (slot != NULL) {
    if (slot->mode == UART_MODE_IT) {
      return HAL_UART_Receive_IT(huart, buf, len);
    } else if (slot->mode == UART_MODE_DMA) {
      HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(huart, buf, len);
      if (status == HAL_OK && huart->hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
      }
      return status;
    }
  }

  return HAL_ERROR;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  UART_Slot_t *slot = slot_get(huart);
  if (slot != NULL) {
    if (slot->mode == UART_MODE_IT && slot->buf != NULL) {
      BSP_UART_RxCallback(huart, slot->buf, slot->len);
    }
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  UART_Slot_t *slot = slot_get(huart);
  if (slot != NULL) {
    if (slot->mode == UART_MODE_DMA && slot->buf != NULL) {
      dcache_invalidate(slot->buf, Size);
      BSP_UART_RxCallback(huart, slot->buf, Size);

      HAL_UARTEx_ReceiveToIdle_DMA(huart, slot->buf, slot->len);
      if (huart->hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
      }
    }
  }
}

HAL_StatusTypeDef BSP_UART_Transmit(UART_HandleTypeDef *huart,
                                    const uint8_t *buf, uint16_t len) {
  UART_Slot_t *slot = slot_get(huart);
  if (slot != NULL) {
    if (slot->mode == UART_MODE_DMA) {
      dcache_clean(buf, len);
      return HAL_UART_Transmit_DMA(huart, (uint8_t *)buf, len);
    } else if (slot->mode == UART_MODE_IT) {
      return HAL_UART_Transmit_IT(huart, (uint8_t *)buf, len);
    } else if (slot->mode == UART_MODE_BASIC) {
      return HAL_UART_Transmit(huart, (uint8_t *)buf, len, HAL_MAX_DELAY);
    }
  }

  return HAL_ERROR;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (slot_get(huart) != NULL) {
    BSP_UART_TxCallback(huart);
  }
}

__weak void BSP_UART_TxCallback(UART_HandleTypeDef *huart) { UNUSED(huart); }

__weak void BSP_UART_RxCallback(UART_HandleTypeDef *huart, uint8_t *buf,
                                uint16_t len) {
  UNUSED(huart);
  UNUSED(buf);
  UNUSED(len);
}