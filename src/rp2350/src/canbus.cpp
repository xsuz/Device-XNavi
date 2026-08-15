#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <Arduino.h>

#include "canbus.h"
#include "sd_logger.h"
#include "clock.h"

#include <SEGGER_RTT.h>
#include <mavlink/swingby/mavlink.h>

namespace canbus
{
    QueueHandle_t canQueue;

    void onPacketReceived(const mavlink_message_t& msg)
    {
        SEGGER_RTT_printf(0, "[%sINFO%s canbus] : Message received.\n", RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET);
        sd_logger::write_pkt(&msg, sys_clock::get_timestamp());
    }

    void task(void *pvParam)
    {
        SEGGER_RTT_printf(0, "[%sINFO%s canbus] : task started.\n",RTT_CTRL_TEXT_GREEN,RTT_CTRL_RESET);
        // TWELITEのUARTを初期化
        SEGGER_RTT_printf(0, "[%sINFO%s canbus] : Initializing Serial2 with RX: 9, TX: 8\n", RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET);
        Serial2.setRX(9);
        Serial2.setTX(8);
        Serial2.setFIFOSize(1024);
        Serial2.begin(115200);
        Serial2.flush();
        canQueue = xQueueCreate(20, sizeof(mavlink_message_t));
        SEGGER_RTT_printf(0, "[%sINFO%s canbus] : Serial2 initialized.\n",RTT_CTRL_TEXT_GREEN,RTT_CTRL_RESET);

        // TWELITEからのデータ受信ループ
        while (true)
        {
            mavlink_message_t msg;
            mavlink_status_t status;
            while (Serial2.available() > 0)
            {
                uint8_t c = Serial2.read();
                if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
                {
                    onPacketReceived(msg);
                }
            }
            while (uxQueueMessagesWaiting(canQueue) > 0)
            {
                if (xQueueReceive(canQueue, &msg, 0) == pdTRUE)
                {
                    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
                    size_t len = mavlink_msg_to_send_buffer(buffer, &msg);
                    for(size_t i = 0; i < len; i++)
                    {
                        Serial2.write(buffer[i]);
                    }
                }
                else
                {
                    SEGGER_RTT_printf(0, "[%sERROR%s canbus] : Failed to receive message from queue.\n", RTT_CTRL_TEXT_RED, RTT_CTRL_RESET);
                }
            }
            vTaskDelay(1); // CPU負荷を下げるために少し待機
        }
    }
    void write_pkt(const mavlink_message_t& pkt)
    {
        if (canQueue != NULL)
        {
            xQueueSend(canQueue, &pkt, 0);
        }
    }
}
