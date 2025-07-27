#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>
#include <queue.h>

#include "gnss.h"
#include "SensorPacket.h"
#include "byte_utils.h"
#include "sd_logger.h"
#include "clock.h"

// #include <ctime>

#include <ubx.h>
#include <SEGGER_RTT.h>

namespace gnss
{
    /// @brief u-blox UBXパーサー
    ubx::parser ubx_parser;
    // std::tm timeinfo;
    volatile uint32_t tick_last_pps = 0;
    uint32_t last_commit_tick = 0;
    QueueHandle_t gnssQueue;
    QueueHandle_t utcQueue;

    void pps_callback(uint gpio, uint32_t emask)
    {
        gpio_set_irq_enabled(gpio, (GPIO_IRQ_EDGE_RISE), false);
        tick_last_pps = millis();
        gpio_set_irq_enabled(gpio, (GPIO_IRQ_EDGE_RISE), true);
    }

    void pvt_callback(ubx::NAV_PVT pvt)
    {
        GPSData data;
        if (pvt.valid.bits.validDate && pvt.valid.bits.validTime && tick_last_pps > 0)
        {
            sys_clock::set_timestamp_offset(tick_last_pps, pvt.year, pvt.month, pvt.day, pvt.hour, pvt.min, pvt.sec);
        }
        int64_t utc = sys_clock::get_timestamp();
        if (pvt.fixType >= 2 && pvt.valid.bits.validTime && pvt.valid.bits.validDate)
        {
            last_commit_tick = ubx_parser.get_last_commit_tick();
            data.id = SensorType::GPS;
            data.latitude = pvt.lat;
            data.longitude = pvt.lon;
            data.altitude = pvt.height;
            data.velN = pvt.velN;
            data.velE = pvt.velE;
            data.velD = pvt.velD;
            data.timestamp = millis();
            data.hAcc = pvt.hAcc;
            data.vAcc = pvt.vAcc;
            data.fixType = pvt.fixType;
            data.pDOP = pvt.pDOP;
            data.flags = pvt.flags.all;

            xQueueSend(gnssQueue, &data, 0);
            xQueueSend(utcQueue, &utc, 0);
        }
    }

    void task(void *pvParam)
    {
        SEGGER_RTT_WriteString(0, "GNSS task started.\n");
        // UART0を初期化
        Serial1.setFIFOSize(2048);
        Serial1.begin(9600);
        delay(1000); // GPSレシーバの起動を待機
        uint8_t cmd0[] = {181, 98, 6, 8, 6, 0, 100, 0, 1, 0, 1, 0, 122, 18};
        Serial1.write(cmd0, sizeof(cmd0)); // RATEを100Hzに設定
        delay(100);
        // NAV-PVT出力を有効化
        uint8_t cmd1[] = {181, 98, 6, 1, 8, 0, 1, 7, 0, 1, 0, 0, 0, 0, 24, 225};
        Serial1.write(cmd1, sizeof(cmd1));
        delay(100);
        // UBX出力を有効化
        uint8_t cmd2[] = {181, 98, 6, 0, 20, 0, 1, 0, 0, 0, 208, 8, 0, 0, 0, 194, 1, 0, 3, 0, 1, 0, 0, 0, 0, 0, 186, 82};
        Serial1.write(cmd2, sizeof(cmd2));
        delay(100);
        Serial1.println("$PUBX,41,1,0007,0003,115200,0*18"); // baudrateを115200に設定
        delay(1000);
        Serial1.flush();       // 無効なデータを破棄
        Serial1.begin(115200); // baudrate 115200で再度UART0を初期化

        gnssQueue = xQueueCreate(5, sizeof(GPSData));
        utcQueue = xQueueCreate(5, sizeof(int64_t));

        if (gnssQueue == NULL)
        {
            SEGGER_RTT_WriteString(0, "Failed to create GNSS queue.\n");
            while (1)
                ; // Halt if queue creation fails
        }
        SEGGER_RTT_WriteString(0, "GNSS queue created successfully.\n");

        ubx_parser.callbackPVT = pvt_callback;

        // PPSによる割り込み設定
        gpio_init(2);
        gpio_set_dir(2, GPIO_IN);
        gpio_set_irq_enabled_with_callback(2, GPIO_IRQ_EDGE_RISE, true, &pps_callback);

        // GNSSデータのポーリングを開始
        xTaskCreate(gnss::task_polling, "gps_polling", 256, NULL, 6, NULL);
        while (1)
        {
            union
            {
                GPSData data;
                uint8_t bytes[sizeof(data)];
            } spkt;
            int64_t utc;
            while (uxQueueMessagesWaiting(gnssQueue) > 0)
            {
                xQueueReceive(gnssQueue, &spkt.data, 0);
                xQueueReceive(utcQueue, &utc, 0);

                // SEGGER_RTT_printf(0, "GNSS Data: Lat: %d, Lon: %d, Alt: %d, VelN: %d, VelE: %d, VelD: %d, hAcc: %u, vAcc: %u, FixType: %u, pDOP: %u\n",
                //                   spkt.data.latitude, spkt.data.longitude, spkt.data.altitude,
                //                   spkt.data.velN, spkt.data.velE, spkt.data.velD,
                //                   spkt.data.hAcc, spkt.data.vAcc,
                //                   spkt.data.fixType, spkt.data.pDOP);

                // バイトオーダーを変換
                swap32<uint32_t>(&spkt.data.id);
                swap32<int32_t>(&spkt.data.latitude);
                swap32<int32_t>(&spkt.data.longitude);
                swap32<int32_t>(&spkt.data.altitude);
                swap32<int32_t>(&spkt.data.velN);
                swap32<int32_t>(&spkt.data.velE);
                swap32<int32_t>(&spkt.data.velD);
                swap32<uint32_t>(&spkt.data.timestamp);
                swap32<uint32_t>(&spkt.data.hAcc);
                swap32<uint32_t>(&spkt.data.vAcc);
                swap16<uint16_t>(&spkt.data.pDOP);
                sd_logger::write_pkt(spkt.bytes, sizeof(spkt.bytes), utc);
            }
            vTaskDelay(100); // 100ms待機
        }
    }

    void task_polling(void *pvParam)
    {
        Serial.begin(115200);
        while (1)
        {
            if (Serial)
            {
                while (Serial.available() > 0)
                {
                    uint8_t c = Serial.read();
                    Serial1.write(c);
                }
            }
            while (Serial1.available() > 0)
            {
                uint8_t c = Serial1.read();
                ubx_parser.parse(c);
                if (Serial)
                {
                    Serial.write(c);
                }
            }
            vTaskDelay(10);
        }
    }
}