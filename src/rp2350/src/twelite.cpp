#include "twelite.h"
#include <FreeRTOS.h>
#include <task.h>
#include <Arduino.h>
#include <PacketSerial.h>
#include "sd_logger.h"

namespace twelite
{
    PacketSerial ps;

    void onPacketReceived(const uint8_t *buffer, size_t size)
    {
        sd_logger::write_pkt(buffer, size);
    }

    void task(void *pvParam)
    {
        // TWELITEのUARTを初期化
        Serial2.setRX(9);
        Serial2.setTX(8);
        Serial2.begin(115200);
        ps.setStream(&Serial2);
        ps.setPacketHandler(&onPacketReceived);

        // TWELITEからのデータ受信ループ
        while (true)
        {
            ps.update(); // 受信データの更新
            vTaskDelay(10); // CPU負荷を下げるために少し待機
        }
    }
}
