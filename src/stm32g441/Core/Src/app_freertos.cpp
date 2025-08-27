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

void cobs_decoding_task(void *argument);

/* USER CODE END FunctionPrototypes */

void default_task(void *argument);

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
    xTaskCreate(default_task, "defaultTask", 128, NULL, osPriorityNormal, &defaultTaskHandle);
    xTaskCreate(cobs_decoding_task, "uartPollingTask", 256, NULL, osPriorityNormal, &uartPollingTaskHandle);
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_default_task */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_default_task */
void default_task(void *argument)
{
    /* USER CODE BEGIN default_task */
    UNUSED(argument);
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
                SEGGER_RTT_printf(0, "Received CAN message: id=0x%X size=%d data=[ ", u.data.id, u.data.size);
                for (size_t i = 0; i < u.data.size; i++)
                {
                    SEGGER_RTT_printf(0, "0x%02X ", u.data.payload[i]);
                }
                SEGGER_RTT_printf(0, "]\n");
                size_t size = cobs::encode(u.raw, u.data.size + 8, encoded_data);
                HAL_UART_Transmit_DMA(&huart2, encoded_data, size);
                while(huart2.gState!=HAL_UART_STATE_READY){}
            }
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // Delay for 5 ms
    }
    /* USER CODE END default_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void cobs_decoding_task(void *argument)
{
    /* USER CODE BEGIN cobs_decoding_task */
    UNUSED(argument);
    /* Infinite loop */
    SEGGER_RTT_printf(0, "UART Polling Task started\n");
    constexpr size_t buf_size = 256;
    size_t start_idx = 0, end_idx = 0;
    uint8_t rx_buf[buf_size], decoded[buf_size];
    union
    {
        DeviceData::CANPacket data;
        uint8_t raw[sizeof(data)];
    } u;
    uint32_t t_threshold = HAL_GetTick()+uart2::USART_RX_BUFFSIZE;
    for (;;)
    {
        while (uart2::available())
        {
            rx_buf[end_idx] = uart2::read();
            end_idx = (end_idx + 1) % buf_size;
            t_threshold = HAL_GetTick()+uart2::USART_RX_BUFFSIZE;
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
                    TxHeader.IdType = FDCAN_STANDARD_ID;
                    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
                    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
                    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
                    TxHeader.MessageMarker = 0;
                    if (u.data.size <= 8)
                    {
                        TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
                        TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
                        TxHeader.DataLength = u.data.size;
                    }
                    else
                    {
                        TxHeader.BitRateSwitch = FDCAN_BRS_ON;
                        TxHeader.FDFormat = FDCAN_FD_CAN;
                        if (u.data.size <= 12)
                        {
                            TxHeader.DataLength = FDCAN_DLC_BYTES_12;
                        }
                        else if (u.data.size <= 16)
                        {
                            TxHeader.DataLength = FDCAN_DLC_BYTES_16;
                        }
                        else if (u.data.size <= 20)
                        {
                            TxHeader.DataLength = FDCAN_DLC_BYTES_20;
                        }
                        else if (u.data.size <= 24)
                        {
                            TxHeader.DataLength = FDCAN_DLC_BYTES_24;
                        }
                        else if (u.data.size <= 32)
                        {
                            TxHeader.DataLength = FDCAN_DLC_BYTES_32;
                        }
                        else if (u.data.size <= 48)
                        {
                            TxHeader.DataLength = FDCAN_DLC_BYTES_48;
                        }
                        else if (u.data.size <= 64)
                        {
                            TxHeader.DataLength = FDCAN_DLC_BYTES_64;
                        }
                    }
                    if (u.data.size <= 64 && u.data.size > 0)
                    {
                        SEGGER_RTT_printf(0, "Sending CAN message: id=0x%X size=%d\n ", u.data.id, u.data.size);
                        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, u.data.payload) != HAL_OK)
                        {
                            Error_Handler();
                        }
                        while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) != 3)
                            ;
                    }

                    SEGGER_RTT_printf(0, "cobs : [ ");
                    for (size_t i = 0; i < size; i++)
                    {
                        SEGGER_RTT_printf(0, "0x%02x ", decoded[i]);
                    }
                    SEGGER_RTT_printf(0, "]\n");
                }
            }
        }

        if (t_threshold < HAL_GetTick())
        {
            uart2::refresh();
            t_threshold = HAL_GetTick()+uart2::USART_RX_BUFFSIZE;
            for (size_t i = 0; i < sizeof(rx_buf); i++)
            {
                rx_buf[i] = 0;
            }
            start_idx = 0;
            end_idx = 0;
        }

        vTaskDelay(1); // Poll every 1000 ms
    }
    /* USER CODE END cobs_decoding_task */
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
        if (fdcan1RxHeader.DataLength <= 8)
        {
            canPacket.size = fdcan1RxHeader.DataLength;
        }
        else
        {
            switch (fdcan1RxHeader.DataLength)
            {
            case FDCAN_DLC_BYTES_12:
                canPacket.size = 12;
                break;
            case FDCAN_DLC_BYTES_16:
                canPacket.size = 16;
                break;
            case FDCAN_DLC_BYTES_20:
                canPacket.size = 20;
                break;
            case FDCAN_DLC_BYTES_24:
                canPacket.size = 24;
                break;
            case FDCAN_DLC_BYTES_32:
                canPacket.size = 32;
                break;
            case FDCAN_DLC_BYTES_48:
                canPacket.size = 48;
                break;
            case FDCAN_DLC_BYTES_64:
                canPacket.size = 64;
                break;
            default:
                canPacket.size = 0;
                break;
            }
        }
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
