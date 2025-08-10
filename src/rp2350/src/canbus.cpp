#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <Arduino.h>
#include <PacketSerial.h>

#include "canbus.h"
#include "sd_logger.h"
#include "clock.h"

#include <SEGGER_RTT.h>

namespace canbus
{
    PacketSerial ps;
    QueueHandle_t canQueue;

    void onPacketReceived(const uint8_t *buffer, size_t size)
    {
        SEGGER_RTT_printf(0, "canbus : size %d [bytes]  packet [ ", size);
        for(int i=0;i<size;i++){
            SEGGER_RTT_printf(0,"0x%02x ",buffer[i]);
        }
        SEGGER_RTT_printf(0,"]\n");
        sd_logger::write_pkt(buffer, size,sys_clock::get_timestamp());
    }

    void task(void *pvParam)
    {
        SEGGER_RTT_WriteString(0, "TWELITE task started.\n");
        // TWELITEのUARTを初期化
        SEGGER_RTT_printf(0, "Initializing Serial2 with RX: %d, TX: %d\n", 9, 8);
        Serial2.setRX(9);
        Serial2.setTX(8);
        Serial2.setFIFOSize(1024);
        Serial2.begin(230400);
        ps.setStream(&Serial2);
        ps.setPacketHandler(&onPacketReceived);
        canQueue = xQueueCreate(20, sizeof(CANPacket));
        SEGGER_RTT_WriteString(0, "TWELITE Serial2 initialized.\n");

        // TWELITEからのデータ受信ループ
        while (true)
        {
            ps.update(); // 受信データの更新
            while (uxQueueMessagesWaiting(canQueue) > 0)
            {
                uint8_t raw[sizeof(CANPacket)];
                if (xQueueReceive(canQueue, &raw, 0) == pdTRUE)
                {
                    ps.send(raw, sizeof(raw)); // パケットを送信
                }
            }
            vTaskDelay(1); // CPU負荷を下げるために少し待機
        }
    }
    void write_pkt(CANPacket pkt)
    {
        if (canQueue != NULL)
        {
            xQueueSend(canQueue, &pkt, 0);
        }
    }
}
