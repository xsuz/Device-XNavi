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
    log_info("task started.\n");
    // TWELITEのUARTを初期化
    log_info("Initializing Serial2 with RX: 9, TX: 8\n");
    Serial2.setRX(9);
    Serial2.setTX(8);
    Serial2.setFIFOSize(1024);
    Serial2.begin(115200);
    Serial2.flush();
    log_info("Serial2 initialized.\n");

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
        if (_subscriber.receive(msg, timestamp,1))
        {
            uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
            size_t len = mavlink_msg_to_send_buffer(buffer, &msg);
            Serial2.write(buffer, len);
        }
    }
}

void CANBusTask::onPacketReceived(const mavlink_message_t &msg)
{
    log_info("Recieved message (msgid:%d, sysid:%d compid:%d)\n",msg.msgid,msg.sysid,msg.compid);
}