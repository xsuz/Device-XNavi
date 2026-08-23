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
#include <SEGGER_RTT.h>
#include "DeviceData.h"
#include "byte_utils.h"

#include <mavlink/swingby/mavlink.h>
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

void decoding_task(void *argument);
void send_message_canfd(const mavlink_message_t *msg);

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
    xTaskCreate(default_task, "defaultTask", 256, NULL, osPriorityNormal, &defaultTaskHandle);
    xTaskCreate(decoding_task, "uartPollingTask", 256, NULL, osPriorityNormal, &uartPollingTaskHandle);
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
        mavlink_canfd_frame_t canfd_msg;
        mavlink_message_t msg;
        if (xQueueReceive(xQueueCANPacketHandle, &canfd_msg, 5) == pdTRUE)
        {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
            SEGGER_RTT_printf(0, "Received CAN message: id=0x%X size=%d data=[ ", canfd_msg.id, canfd_msg.len);
            for (size_t i = 0; i < canfd_msg.len; i++)
            {
                SEGGER_RTT_printf(0, "0x%02X ", canfd_msg.data[i]);
            }
            SEGGER_RTT_printf(0, "]\n");
            mavlink_msg_canfd_frame_encode(1, 1, &msg, &canfd_msg);
            uint8_t encoded_data[MAVLINK_MAX_PACKET_LEN];
            size_t size = mavlink_msg_to_send_buffer(encoded_data, &msg);
            HAL_UART_Transmit_DMA(&huart2, encoded_data, size);
            while (huart2.gState != HAL_UART_STATE_READY)
            {
            }
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        }
    }
    /* USER CODE END default_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void decoding_task(void *argument)
{
    /* USER CODE BEGIN cobs_decoding_task */
    UNUSED(argument);
    /* Infinite loop */
    SEGGER_RTT_printf(0, "UART Polling Task started\n");

    uint32_t t_threshold = HAL_GetTick() + uart2::USART_RX_BUFFSIZE;
    for (;;)
    {
        while (uart2::available())
        {
            mavlink_message_t msg;
            t_threshold = HAL_GetTick() + uart2::USART_RX_BUFFSIZE;
            if (mavlink_parse_char(MAVLINK_COMM_0, uart2::read(), &msg, NULL))
            {
                SEGGER_RTT_printf(0, "[%sINFO%s decoding_task] recieved msg (msgid=0x%02x,sysid=0x%02x,compid=0x%02x\n", RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET, msg.msgid, msg.sysid, msg.compid);
                send_message_canfd(&msg);
            }
        }

        if (t_threshold < HAL_GetTick())
        {
            uart2::refresh();
            t_threshold = HAL_GetTick() + uart2::USART_RX_BUFFSIZE;
        }

        vTaskDelay(1); // Poll every 1000 ms
    }
    /* USER CODE END cobs_decoding_task */
}

void send_message_canfd(const mavlink_message_t *msg)
{
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    size_t message_length = mavlink_msg_to_send_buffer(buffer, msg);

    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = msg->msgid;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    TxHeader.BitRateSwitch = FDCAN_BRS_ON;
    TxHeader.FDFormat = FDCAN_FD_CAN;
    if (message_length <= 12)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_12;
    }
    else if (message_length <= 16)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_16;
    }
    else if (message_length <= 20)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_20;
    }
    else if (message_length <= 24)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_24;
    }
    else if (message_length <= 32)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_32;
    }
    else if (message_length <= 48)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_48;
    }
    else if (message_length <= 64)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_64;
    }
    if (message_length <= 64 && message_length > 0)
    {
        SEGGER_RTT_printf(0, "Sending CAN message: id=0x%X size=%d\n ", msg->msgid, message_length);
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, buffer) != HAL_OK)
        {
            Error_Handler();
        }
        while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) != 3)
            ;
    }
    else
    {
        SEGGER_RTT_printf(0, "Message too long to send over CAN: id=0x%X size=%d\n ", msg->msgid, message_length);
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef fdcan1RxHeader;
    uint8_t fdcan1RxData[64];
    mavlink_canfd_frame_t canfd_frame;

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &fdcan1RxHeader, fdcan1RxData) != HAL_OK)
        {
            /* Reception Error */
            Error_Handler();
        }
        canfd_frame.id = fdcan1RxHeader.Identifier;
        if (fdcan1RxHeader.DataLength <= 8)
        {
            canfd_frame.len = fdcan1RxHeader.DataLength;
        }
        else
        {
            switch (fdcan1RxHeader.DataLength)
            {
            case FDCAN_DLC_BYTES_12:
                canfd_frame.len = 12;
                break;
            case FDCAN_DLC_BYTES_16:
                canfd_frame.len = 16;
                break;
            case FDCAN_DLC_BYTES_20:
                canfd_frame.len = 20;
                break;
            case FDCAN_DLC_BYTES_24:
                canfd_frame.len = 24;
                break;
            case FDCAN_DLC_BYTES_32:
                canfd_frame.len = 32;
                break;
            case FDCAN_DLC_BYTES_48:
                canfd_frame.len = 48;
                break;
            case FDCAN_DLC_BYTES_64:
                canfd_frame.len = 64;
                break;
            default:
                canfd_frame.len = 0;
                break;
            }
        }
        for (size_t i = 0; i < canfd_frame.len; i++)
        {
            canfd_frame.data[i] = fdcan1RxData[i];
        }

        xQueueSendFromISR(xQueueCANPacketHandle, &canfd_frame, NULL);

        if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
        {
            Error_Handler();
        }
    }
}

/* USER CODE END Application */
