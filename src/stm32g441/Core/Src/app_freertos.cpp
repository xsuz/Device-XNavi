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
#include "cobs.h"
#include "SEGGER_RTT.h"
#include "DeviceData.h"
#include "byte_utils.h"
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
TaskHandle_t uartPollingTaskHandle;

/* USER CODE END Variables */
/* Definitions for defaultTask */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void StartUartPollingTask(void *argument);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
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

    xQueueCANPacketHandle = xQueueCreate(10, sizeof(DeviceData::CANPacket));
    if (xQueueCANPacketHandle == NULL)
    {
        Error_Handler();
    }
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */

    /* USER CODE BEGIN RTOS_THREADS */
    xTaskCreate(StartDefaultTask, "defaultTask", 128, NULL, osPriorityNormal, &defaultTaskHandle);
    xTaskCreate(StartUartPollingTask, "uartPollingTask", 128, NULL, osPriorityNormal, &uartPollingTaskHandle);
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
            DeviceData::CANPacket data;
            uint8_t raw[sizeof(DeviceData::CANPacket)];
        } u;
        uint8_t encoded_data[sizeof(u.data) + 2];

        while (uxQueueMessagesWaiting(xQueueCANPacketHandle) > 0)
        {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
            if (xQueueReceive(xQueueCANPacketHandle, &u.data, portMAX_DELAY) == pdTRUE)
            {
                SEGGER_RTT_printf(0, "Received CAN message: id=0x%X data=[ ", u.data.id);
                for (size_t i = 0; i < u.data.size; i++)
                {
                    SEGGER_RTT_printf(0, "0x%02X ", u.data.payload[i]);
                }
                SEGGER_RTT_printf(0, "]\n");
                size_t size = cobs::encode(u.raw, u.data.size + 8, encoded_data);
                HAL_UART_Transmit(&huart2, encoded_data, size, HAL_MAX_DELAY);
            }
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Delay for 10 ms
    }
    /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void StartUartPollingTask(void *argument)
{
    /* USER CODE BEGIN StartUartPollingTask */
    /* Infinite loop */
    SEGGER_RTT_printf(0, "UART Polling Task started\n");
    constexpr size_t buf_size = 256;
    uint8_t rx_buf[buf_size], decoded[128];
    union
    {
        DeviceData::CANPacket data;
        uint8_t raw[sizeof(data)];
    } u;
    size_t start_idx = 0, end_idx = 0;
    for (;;)
    {
        while (available())
        {
            rx_buf[end_idx] = read();
            end_idx = (end_idx + 1) % buf_size;
            size_t size = cobs::decode(rx_buf, buf_size, &start_idx, end_idx, decoded);
            if (size > 0)
            {
                if (size <= sizeof(u.raw))
                {
                    for (size_t i = 0; i < size; i++)
                    {
                        u.raw[i] = decoded[i];
                    }

                    FDCAN_TxHeaderTypeDef TxHeader;
                    TxHeader.Identifier = u.data.id;
                    TxHeader.DataLength = u.data.size;
                    TxHeader.IdType = FDCAN_STANDARD_ID;
                    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
                    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
                    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
                    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
                    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
                    TxHeader.MessageMarker = 0;
                    if (u.data.size <= 64)
                    {
                        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, u.data.payload) != HAL_OK)
                        {
                            Error_Handler();
                        }
                        while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) != 3)
                            ;
                    }
                }
                SEGGER_RTT_printf(0, "cobs : [ ");
                for (size_t i = 0; i < size; i++)
                {
                    SEGGER_RTT_printf(0, "0x%02x ", decoded[i]);
                }
                SEGGER_RTT_printf(0, "]\n");
            }
        }
        vTaskDelay(10); // Poll every 1000 ms
    }
    /* USER CODE END StartUartPollingTask */
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef fdcan1RxHeader;
    uint8_t fdcan1RxData[64];
    DeviceData::CANPacket canPacket;

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &fdcan1RxHeader, fdcan1RxData) != HAL_OK)
        {
            /* Reception Error */
            Error_Handler();
        }
        canPacket.id = fdcan1RxHeader.Identifier;
        canPacket.size = fdcan1RxHeader.DataLength;
        for (size_t i = 0; i < canPacket.size; i++)
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

/* USER CODE END Application */
