/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpdma.h"
#include "gpio.h"
#include "usart.h"
#include "xspim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_led.h"
#include "bsp_uart.h"
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */
extern DMA_HandleTypeDef handle_GPDMA1_Channel0;
extern DMA_HandleTypeDef handle_GPDMA1_Channel1;

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
__attribute__((aligned(32))) uint8_t received_data[128];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void SystemIsolation_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_USART1_UART_Init();
  SystemIsolation_Config();
  /* USER CODE BEGIN 2 */
  // char msg[] = "Hello from Secure World!\r\n";
  // HAL_UART_Receive_IT(&huart1, received_data, 2);
  // HAL_UARTEx_ReceiveToIdle_DMA(&huart1, received_data,
  // sizeof(received_data));
  // __HAL_DMA_DISABLE_IT(&handle_GPDMA1_Channel0, DMA_IT_HT);

  BSP_LED_Init(LED_RED);
  BSP_LED_Init(LED_GREEN);
  BSP_UART_Init_DMA(&huart1, received_data, sizeof(received_data));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    // HAL_UART_Transmit_IT(&huart1, (uint8_t*)msg, strlen(msg));
    // HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief RIF Initialization Function
 * @param None
 * @retval None
 */
static void SystemIsolation_Config(void) {

  /* USER CODE BEGIN RIF_Init 0 */

  /* USER CODE END RIF_Init 0 */

  /* set all required IPs as secure privileged */
  __HAL_RCC_RIFSC_CLK_ENABLE();

  /* RIF-Aware IPs Config */

  /* set up PWR configuration */
  HAL_PWR_ConfigAttributes(PWR_ITEM_0, PWR_SEC_NPRIV);

  /* set up GPDMA configuration */
  /* set GPDMA1 channel 0 used by USART1 */
  if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel0,
                                      DMA_CHANNEL_SEC | DMA_CHANNEL_PRIV |
                                        DMA_CHANNEL_SRC_SEC |
                                        DMA_CHANNEL_DEST_SEC) != HAL_OK) {
    Error_Handler();
  }
  /* set GPDMA1 channel 1 used by USART1 */
  if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel1,
                                      DMA_CHANNEL_SEC | DMA_CHANNEL_PRIV |
                                        DMA_CHANNEL_SRC_SEC |
                                        DMA_CHANNEL_DEST_SEC) != HAL_OK) {
    Error_Handler();
  }

  /* set up GPIO configuration */
  HAL_GPIO_ConfigPinAttributes(GPIOE, GPIO_PIN_5,
                               GPIO_PIN_SEC | GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOE, GPIO_PIN_6,
                               GPIO_PIN_SEC | GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOE, GPIO_PIN_10,
                               GPIO_PIN_SEC | GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOG, GPIO_PIN_10,
                               GPIO_PIN_SEC | GPIO_PIN_NPRIV);

  /* USER CODE BEGIN RIF_Init 1 */

  /* USER CODE END RIF_Init 1 */
  /* USER CODE BEGIN RIF_Init 2 */

  /* USER CODE END RIF_Init 2 */
}

/* USER CODE BEGIN 4 */

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//   if (huart == &huart1) {
//     HAL_UART_Transmit_IT(huart, received_data, 2);

//     GPIO_PinState state =
//       (received_data[1] == '1') ? GPIO_PIN_RESET : GPIO_PIN_SET;

//     if (received_data[0] == 'R') {
//       HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, state);
//     } else if (received_data[0] == 'G') {
//       HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, state);
//     }
//     HAL_UART_Receive_IT(huart, received_data, 2);
//   }
// }

// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
//   if (huart == &huart1) {
//     // important: invalidate the D-Cache for the received buffer to ensure
//     SCB_InvalidateDCache_by_Addr(received_data, sizeof(received_data));
//     SCB_CleanDCache_by_Addr(received_data, sizeof(received_data));

//     HAL_UART_Transmit_DMA(&huart1, received_data, Size);
//     HAL_UARTEx_ReceiveToIdle_DMA(&huart1, received_data,
//     sizeof(received_data));
//     __HAL_DMA_DISABLE_IT(&handle_GPDMA1_Channel0, DMA_IT_HT);
//   }
// }

void BSP_UART_RxCallback(UART_HandleTypeDef *huart, uint8_t *buf,
                         uint16_t len) {
  if (len > 5 && strncmp((char *)buf, "CMD: ", 5) == 0) {
    uint8_t *data = buf + 5;
    for (int i = 0; i < ((len - 5) / 2); ++i) {
      LedState_TypeDef state = (data[i << 1 | 1] == '1') ? LED_ON : LED_OFF;

      if (data[i << 1] == 'R') {
        BSP_LED_SetState(LED_RED, state);
      } else if (data[i << 1] == 'G') {
        BSP_LED_SetState(LED_GREEN, state);
      }

      HAL_Delay(200);
    }
  }

  BSP_UART_Transmit(huart, buf, len);
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
