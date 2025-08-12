/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : app_freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "fdcan.h"
#include "usart.h"
#include "SEGGER_RTT.h"
#include "SensorPacket.h"
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

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

QueueHandle_t xQueueCANPacketHandle;
TaskHandle_t defaultTaskHandle;

/* USER CODE END Variables */
/* Definitions for defaultTask */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/// @brief Send packet via UART with COBS encoding
/// @param data Pointer to the data to send
/// @param size Size of the data to send
void send_pkt(uint8_t *data, uint32_t size);

void StartUartPollingTask(void *argument);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */

  xQueueCANPacketHandle = xQueueCreate(10, sizeof(struct CANPacket));
  if (xQueueCANPacketHandle == NULL)
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */

  /* USER CODE BEGIN RTOS_THREADS */
  xTaskCreate(StartDefaultTask,"defaultTask",128,NULL,osPriorityNormal,&defaultTaskHandle);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;)
  {
    union
    {
      struct CANPacket data;
      uint8_t raw[sizeof(struct CANPacket)];
    } u;

    while (uxQueueMessagesWaiting(xQueueCANPacketHandle) > 0)
    {
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      if (xQueueReceive(xQueueCANPacketHandle, &u.data, portMAX_DELAY) == pdTRUE)
      {
        SEGGER_RTT_printf(0, "Received CAN message: id=0x%X data=[ ", u.data.id);
        for (int32_t i = 0; i < u.data.size; i++)
        {
          SEGGER_RTT_printf(0, "0x%02X ", u.data.payload[i]);
        }
        SEGGER_RTT_printf(0, "]\n");
        send_pkt(u.raw, sizeof(u.raw));
      }
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    }
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void StartUartPollingTask(void *argument)
{
  /* USER CODE BEGIN StartUartPollingTask */
  /* Infinite loop */
  for (;;)
  {
  }
  /* USER CODE END StartUartPollingTask */
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef fdcan1RxHeader;
  uint8_t fdcan1RxData[64];
  struct CANPacket canPacket;
  
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &fdcan1RxHeader, fdcan1RxData) != HAL_OK)
    {
      /* Reception Error */
      Error_Handler();
    }
    canPacket.id = fdcan1RxHeader.Identifier;
    canPacket.size = fdcan1RxHeader.DataLength;
    if (canPacket.size > 8)
    {
      canPacket.size = 8; // Limit size to 64 bytes
    }
    for (int32_t i = 0; i < canPacket.size; i++)
    {
      canPacket.payload[i] = fdcan1RxData[i];
    }

    xQueueSendFromISR(xQueueCANPacketHandle, &canPacket, NULL);

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
      Error_Handler();
    }
  }
}

void send_pkt(uint8_t *data, uint32_t size)
{
  // COBS encoding and sending logic here
  // For now, just print the data to RTT

  uint8_t cobs_buf_idx = 0, encoded_idx = 0;
  uint8_t cobs_buf[256], encoded_data[256];

  // COBS encoding
  for (uint32_t i = 0; i < size; i++)
  {
    if (data[i] == 0)
    {
      encoded_data[encoded_idx] = cobs_buf_idx + 1;
      encoded_idx++;
      for (uint8_t j = 0; j < cobs_buf_idx; j++)
      {
        encoded_data[encoded_idx] = cobs_buf[j];
        encoded_idx++;
      }
      cobs_buf_idx = 0; // Reset for next segment
    }
    else
    {
      cobs_buf[cobs_buf_idx] = data[i];
      cobs_buf_idx++;
    }
  }
  encoded_data[encoded_idx++] = cobs_buf_idx + 1; // Final segment length
  for (uint8_t j = 0; j < cobs_buf_idx; j++)
  {
    encoded_data[encoded_idx++] = cobs_buf[j];
  }
  encoded_data[encoded_idx++] = 0; // End of COBS encoding
  // Send the COBS-encoded packet over UART
  HAL_UART_Transmit(&huart2, encoded_data, encoded_idx, HAL_MAX_DELAY);
}

/* USER CODE END Application */

