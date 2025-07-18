#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>

#include "gnss.h"
#include "SensorPacket.h"
#include "byte_utils.h"
#include "sd_logger.h"

// #include <ctime>

#include <ubx.h>
#include <SEGGER_RTT.h>

namespace gnss
{
    /// @brief u-blox UBXパーサー
    ubx::parser ubx_parser;
    // std::tm timeinfo;
    int64_t t = 0;
    int64_t offset = 0;

    void pps_callback(uint gpio, uint32_t emask)
    {
        gpio_set_irq_enabled(gpio, (GPIO_IRQ_EDGE_RISE), false);
        if (ubx_parser.get_nav_pvt().valid.bits.validTime && ubx_parser.get_nav_pvt().valid.bits.validDate && ubx_parser.age()<1000)
        {
            sd_logger::set_timestamp_offset(ubx_parser.get_nav_pvt().year, ubx_parser.get_nav_pvt().month, ubx_parser.get_nav_pvt().day, ubx_parser.get_nav_pvt().hour, ubx_parser.get_nav_pvt().min, ubx_parser.get_nav_pvt().sec);
            // sd_logger::set_timestamp_offset(((((gps.time.hour() + 9) % 24) * 60 + gps.time.minute()) * 60 + gps.time.second() + 1) * 1000 - millis());
        }
        gpio_set_irq_enabled(gpio, (GPIO_IRQ_EDGE_RISE), true);
    }

    void task(void *pvParam)
    {
        SEGGER_RTT_WriteString(0, "GNSS task started.\n");
        // UART0を初期化
        Serial1.setFIFOSize(2048);
        Serial1.begin(9600);
        delay(1000);                           // GPSレシーバの起動を待機
        uint8_t cmd0[]={181, 98, 6, 8, 6, 0, 100, 0, 1, 0, 1, 0, 122, 18};
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

        // PPSによる割り込み設定
        gpio_init(2);
        gpio_set_dir(2, GPIO_IN);
        gpio_set_irq_enabled_with_callback(2, GPIO_IRQ_EDGE_RISE, true, &pps_callback);

        auto timerGNSS = xTimerCreate("gnss", 100, pdTRUE, 0, timer_callback);
        xTimerStart(timerGNSS, 0);

        while (1)
        {
            while(Serial.available() > 0)
            {
                uint8_t c = Serial.read();
                Serial1.write(c);
            }
            while (Serial1.available() > 0)
            {
                uint8_t c = Serial1.read();
                ubx_parser.parse(c);
                Serial.write(c);
            }
            vTaskDelay(1);
        }
    }

    void timer_callback(TimerHandle_t xTimer)
    {
        union
        {
            GPSData data;
            uint8_t bytes[sizeof(data)];
        } spkt;
        if (ubx_parser.get_nav_pvt().fixType>=2)
        {
            spkt.data.id = 0x60;
            spkt.data.latitude = ubx_parser.get_nav_pvt().lat;
            spkt.data.longitude = ubx_parser.get_nav_pvt().lon;
            spkt.data.altitude = ubx_parser.get_nav_pvt().height;
            spkt.data.velN = ubx_parser.get_nav_pvt().velN;
            spkt.data.velE = ubx_parser.get_nav_pvt().velE;
            spkt.data.velD = ubx_parser.get_nav_pvt().velD;
            spkt.data.timestamp = millis();
            spkt.data.hAcc = ubx_parser.get_nav_pvt().hAcc;
            spkt.data.vAcc = ubx_parser.get_nav_pvt().vAcc;
            spkt.data.fixType = ubx_parser.get_nav_pvt().fixType;
            spkt.data.pDOP = ubx_parser.get_nav_pvt().pDOP;
            spkt.data.flags =ubx_parser.get_nav_pvt().flags.all;
            swap32<int32_t>(&spkt.data.latitude);
            swap32<int32_t>(&spkt.data.longitude);
            swap32<int32_t>(&spkt.data.altitude);
            swap32<int32_t>(&spkt.data.velN);
            swap32<int32_t>(&spkt.data.velE);
            swap32<int32_t>(&spkt.data.velD);
            swap32<uint32_t>(&spkt.data.timestamp);
            swap32<uint32_t>(&spkt.data.hAcc);
            swap32<uint32_t>(&spkt.data.vAcc);

            SEGGER_RTT_printf(0, "GNSS Data: Lat: %d, Lon: %d, Alt: %d, VelN: %d, VelE: %d, VelD: %d, hAcc: %u, vAcc: %u, FixType: %u, pDOP: %u\n",
                spkt.data.latitude, spkt.data.longitude, spkt.data.altitude,
                spkt.data.velN, spkt.data.velE, spkt.data.velD,
                spkt.data.hAcc, spkt.data.vAcc,
                spkt.data.fixType, spkt.data.pDOP);
            sd_logger::write_pkt(spkt.bytes, sizeof(spkt.bytes));
        }
    }
}