#include "bsp_uart.h"
#include <stdint.h>

#define BSP_UART_MAX    2u
#define UART_MODE_BASIC 0u
#define UART_MODE_IT    1u
#define UART_MODE_DMA   2u

typedef struct UART_Slot {
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

HAL_StatusTypeDef BSP_UART_Init_IT(UART_HandleTypeDef *huart, uint8_t *buf,
                                   uint16_t len) {
  slot_add(huart, buf, len, UART_MODE_IT);
  return HAL_UART_Receive_IT(huart, buf, len);
}

HAL_StatusTypeDef BSP_UART_Init_DMA(UART_HandleTypeDef *huart, uint8_t *buf,
                                    uint16_t max_len) {
  slot_add(huart, buf, max_len, UART_MODE_DMA);

  HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(huart, buf, max_len);
  if (status == HAL_OK && huart->hdmarx != NULL) {
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
  return status;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  UART_Slot_t *slot = slot_get(huart);
  if (slot != NULL) {
    if (slot->mode == UART_MODE_IT) {
      BSP_UART_RxCallback(huart, slot->buf, slot->len);
    }
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  UART_Slot_t *slot = slot_get(huart);
  if (slot != NULL) {
    if (slot->mode == UART_MODE_DMA) {
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