#include "bsp_key.h"

static GPIO_TypeDef *KEY_PORT[KEYn] = {
  BSP_KEY_UP_GPIO_PORT, BSP_KEY_DOWN_GPIO_PORT, BSP_KEY_LEFT_GPIO_PORT,
  BSP_KEY_RIGHT_GPIO_PORT};

static const uint16_t KEY_PIN[KEYn] = {
  BSP_KEY_UP_GPIO_PIN, BSP_KEY_DOWN_GPIO_PIN, BSP_KEY_LEFT_GPIO_PIN,
  BSP_KEY_RIGHT_GPIO_PIN};

uint32_t BSP_KEY_GetState(Key_TypeDef Key) {
  return (uint32_t)HAL_GPIO_ReadPin(KEY_PORT[Key], KEY_PIN[Key]);
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
  static uint32_t last_tick = 0;
  uint32_t current_tick = HAL_GetTick();
  if (current_tick - last_tick < KEY_DEBOUNCE_TIME_MS)
    return;
  last_tick = current_tick;

  switch (GPIO_Pin) {
  case BSP_KEY_LEFT_GPIO_PIN:
    BSP_KEY_Callback(KEY_LEFT);
    break;
  case BSP_KEY_DOWN_GPIO_PIN:
    BSP_KEY_Callback(KEY_DOWN);
    break;
  case BSP_KEY_RIGHT_GPIO_PIN:
    BSP_KEY_Callback(KEY_RIGHT);
    break;
  default:
    break;
  }
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
  static uint32_t last_tick = 0;
  uint32_t current_tick = HAL_GetTick();
  if (current_tick - last_tick < KEY_DEBOUNCE_TIME_MS)
    return;
  last_tick = current_tick;

  switch (GPIO_Pin) {
  case BSP_KEY_UP_GPIO_PIN:
    BSP_KEY_Callback(KEY_UP);
    break;
  default:
    break;
  }
}
__weak void BSP_KEY_Callback(Key_TypeDef Key) {
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Key);
  /* This function should be implemented by the user application.
     It is called into this driver when an event on Key is triggered. */
}
