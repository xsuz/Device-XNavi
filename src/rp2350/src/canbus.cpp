#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <Arduino.h>

#include "canbus.h"
#include "clock.h"

#include <SEGGER_RTT.h>
#include <mavlink/swingby/mavlink.h>

void CANBusTask::run()
{
    SEGGER_RTT_printf(0, "[%sINFO%s CANBus] : task started.\n", RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET);
    // TWELITEのUARTを初期化
    SEGGER_RTT_printf(0, "[%sINFO%s CANBus] : Initializing Serial2 with RX: 9, TX: 8\n", RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET);
    Serial2.setRX(9);
    Serial2.setTX(8);
    Serial2.setFIFOSize(1024);
    Serial2.begin(115200);
    Serial2.flush();
    SEGGER_RTT_printf(0, "[%sINFO%s CANBus] : Serial2 initialized.\n", RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET);

    // TWELITEからのデータ受信ループ
    while (true)
    {
        mavlink_message_t msg;
        int64_t timestamp;
        mavlink_status_t status;
        while (Serial2.available() > 0)
        {
            uint8_t c = Serial2.read();
            if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
            {
                onPacketReceived(msg);
            }
        }
        if (_consumer.receive(msg, timestamp))
        {
            uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
            size_t len = mavlink_msg_to_send_buffer(buffer, &msg);
            Serial2.write(buffer, len);
        }
        vTaskDelay(1); // CPU負荷を下げるために少し待機
    }
}

void CANBusTask::onPacketReceived(const mavlink_message_t &msg)
{
    SEGGER_RTT_printf(0, "[%sINFO%s CANBus] : Message received.\n", RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET);
}