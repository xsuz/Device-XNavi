#include <FreeRTOS.h>
#include <task.h>
#include <Arduino.h>
#include <PacketSerial.h>

#include "twelite.h"
#include "sd_logger.h"

#include <SEGGER_RTT.h>

namespace twelite
{
    PacketSerial ps;

    void onPacketReceived(const uint8_t *buffer, size_t size)
    {
        SEGGER_RTT_printf(0, "Received packet size: %d bytes : [ ", size);
        for(int i=0;i<size;i++){
            SEGGER_RTT_printf(0,"%02x ",buffer[i]);
        }
        SEGGER_RTT_printf(0," ]\n");
        sd_logger::write_pkt(buffer, size);
    }

    void task(void *pvParam)
    {
        SEGGER_RTT_WriteString(0, "TWELITE task started.\n");
        // TWELITEのUARTを初期化
        SEGGER_RTT_printf(0, "Initializing Serial2 with RX: %d, TX: %d\n", 9, 8);
        Serial2.setRX(9);
        Serial2.setTX(8);
        Serial2.begin(115200);
        ps.setStream(&Serial2);
        ps.setPacketHandler(&onPacketReceived);
        SEGGER_RTT_WriteString(0, "TWELITE Serial2 initialized.\n");

        // TWELITEからのデータ受信ループ
        while (true)
        {
            ps.update(); // 受信データの更新
            vTaskDelay(10); // CPU負荷を下げるために少し待機
        }
    }
}
