#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>
#include <queue.h>

#include "gnss.h"
#include "sd_logger.h"
#include "clock.h"
#include "canbus.h"

// #include <ctime>

#include <ubx.h>
#include <SEGGER_RTT.h>
#include <mavlink/swingby/mavlink.h>

namespace gnss
{
    /// @brief u-blox UBXパーサー
    ubx::parser ubx_parser;
    /// @brief PPSによる割込みが発生した時刻
    volatile uint32_t tick_last_pps = 0;
    QueueHandle_t gnssQueue;
    QueueHandle_t utcQueue;

    constexpr int LED = 11;

    void pps_callback(uint gpio, uint32_t emask)
    {
        gpio_set_irq_enabled(gpio, (GPIO_IRQ_EDGE_RISE), false);
        tick_last_pps = millis();
        gpio_set_irq_enabled(gpio, (GPIO_IRQ_EDGE_RISE), true);
    }

    void pvt_callback(ubx::NAV_PVT pvt)
    {
        digitalWrite(LED, HIGH);
        if (pvt.valid.bits.validDate && pvt.valid.bits.validTime && tick_last_pps > 0)
        {
            sys_clock::set_timestamp_offset(tick_last_pps, pvt.year, pvt.month, pvt.day, pvt.hour, pvt.min, pvt.sec);
        }
        int64_t utc = sys_clock::get_timestamp();

        SEGGER_RTT_printf(0, "[%sINFO%s gnss] : latitude: %d, longitude: %d, altitude: %d, velN: %d, velE: %d, velD: %d, hAcc: %u, vAcc: %u, fixType: %u, pDOP: %u\n",
                        RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET,
                        pvt.lat, pvt.lon, pvt.height, pvt.velN, pvt.velE, pvt.velD, pvt.hAcc, pvt.vAcc, pvt.fixType, pvt.pDOP);

        xQueueSend(gnssQueue, &pvt, 0);
        xQueueSend(utcQueue, &utc, 0);

        digitalWrite(LED, LOW);
    }

    void callback_reset()
    {
        Serial1.begin(115200);
        Serial1.flush();
        SEGGER_RTT_printf(0, "[%sWARN%s gnss] : reset UBX parser.\n", RTT_CTRL_TEXT_YELLOW, RTT_CTRL_RESET);
    }

    void task(void *pvParam)
    {
        int64_t utc;
        ubx::NAV_PVT pvt;
        mavlink_message_t msg;
        mavlink_gps_raw_int_t gps_raw;

        SEGGER_RTT_printf(0, "[%sINFO%s gnss] : GNSS task started.\n",RTT_CTRL_TEXT_GREEN,RTT_CTRL_RESET);
        pinMode(LED, OUTPUT);
        digitalWrite(LED, LOW);
        // UART0を初期化
        Serial1.setFIFOSize(2048);
        Serial1.begin(9600);
        delay(1000); // GPSレシーバの起動を待機
        const uint8_t UBX_HEADER1=0xb5,UBX_HEADER2=0x62,UBX_CFG=0x06;

        // [0xB5 0x62] : UBX header, 0x06 : class=UBX-CFG, 0x08 : message ID=CFG-RATE, [0x06 0x00] : payload length, [0x64 0x00] : measRate=100ms, [0x01 0x00] : navRate=1, [0x01 0x00] : timeRef=1, [0x7A 0x12] : checksum
        uint8_t cmd0[] = {UBX_HEADER1, UBX_HEADER2, 0x06, 0x08, 6, 0, 100, 0, 1, 0, 1, 0, 122, 18};
        Serial1.write(cmd0, sizeof(cmd0)); // RATEを10Hzに設定
        delay(100);
        // NAV-PVT出力を有効化
        uint8_t cmd1[] = {UBX_HEADER1, UBX_HEADER2, 0x06, 1, 8, 0, 1, 7, 0, 1, 0, 0, 0, 0, 24, 225};
        Serial1.write(cmd1, sizeof(cmd1));
        delay(100);
        // UBX出力を有効化
        uint8_t cmd2[] = {UBX_HEADER1, UBX_HEADER2, 0x06, 0, 20, 0, 1, 0, 0, 0, 208, 8, 0, 0, 0, 194, 1, 0, 3, 0, 1, 0, 0, 0, 0, 0, 186, 82};
        Serial1.write(cmd2, sizeof(cmd2));
        delay(100);
        // PPSの基準を
        Serial1.println("$PUBX,41,1,0007,0003,115200,0*18"); // baudrateを115200に設定
        delay(1000);
        Serial1.flush();       // 無効なデータを破棄
        Serial1.begin(115200); // baudrate 115200で再度UART0を初期化

        gnssQueue = xQueueCreate(5, sizeof(ubx::NAV_PVT));
        utcQueue = xQueueCreate(5, sizeof(int64_t));

        if (gnssQueue == NULL)
        {
            SEGGER_RTT_printf(0, "[%sERROR%s gnss] : Failed to create GNSS queue.\n", RTT_CTRL_TEXT_RED, RTT_CTRL_RESET);
            while (1)
                ; // Halt if queue creation fails
        }
        SEGGER_RTT_printf(0, "[%sINFO%s gnss] : GNSS queue created successfully.\n", RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET);

        ubx_parser.callbackPVT = pvt_callback;
        ubx_parser.callbackReset = callback_reset;

        // PPSによる割り込み設定
        gpio_init(2);
        gpio_set_dir(2, GPIO_IN);
        gpio_set_irq_enabled_with_callback(2, GPIO_IRQ_EDGE_RISE, true, &pps_callback);

        // GNSSデータのポーリングを開始
        xTaskCreate(gnss::task_polling, "gps_polling", 256, NULL, 6, NULL);
        while (1)
        {
            while (uxQueueMessagesWaiting(gnssQueue) > 0)
            {
                xQueueReceive(gnssQueue, &pvt, 0);
                xQueueReceive(utcQueue, &utc, 0);

                gps_raw.time_usec = utc;
                gps_raw.lat = pvt.lat;
                gps_raw.lon = pvt.lon;
                gps_raw.alt = pvt.hMSL;
                gps_raw.eph = pvt.hAcc;
                gps_raw.epv = pvt.vAcc;
                gps_raw.vel = pvt.gSpeed*10;
                gps_raw.cog = pvt.headMot;
                gps_raw.fix_type = pvt.fixType;
                gps_raw.satellites_visible = pvt.numSV;
                gps_raw.alt_ellipsoid = pvt.height;
                gps_raw.h_acc = pvt.hAcc;
                gps_raw.v_acc = pvt.vAcc;
                gps_raw.vel_acc = pvt.sAcc;
                gps_raw.hdg_acc = pvt.headAcc;
                gps_raw.yaw = 0;

                mavlink_msg_gps_raw_int_encode(1, 0, &msg, &gps_raw);
                sd_logger::write_pkt(&msg, utc);

                canbus::write_pkt(msg);
                
            }
            vTaskDelay(10); // 10ms待機
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